#include <camera_projection.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

// 检查点是否在相机前方
static bool IsInFrontOfCamera(const Point3D& point)
{
    return point.z > 0.0;
}

// 归一化平面坐标（针孔投影）
static void NormalizedProjection(
    const Point3D& point,
    double& out_x_norm,
    double& out_y_norm)
{
    out_x_norm = point.x / point.z;
    out_y_norm = point.y / point.z;
}

// ---------------------------------------------------------------------------
// 完整畸变（径向+切向）及其雅可比矩阵
// 用于牛顿迭代法求逆畸变
// ---------------------------------------------------------------------------
static void DistortionWithJacobian(
    double x, double y,
    const DistortionCoefficients& dist,
    double& out_x_dist, double& out_y_dist,
    double& Jxx, double& Jxy,
    double& Jyx, double& Jyy)
{
    double r2  = x * x + y * y;
    double r4  = r2 * r2;
    double r6  = r4 * r2;

    double rf  = 1.0 + dist.k1 * r2 + dist.k2 * r4 + dist.k3 * r6;  // radial_factor
    double drf = 2.0 * dist.k1 + 4.0 * dist.k2 * r2 + 6.0 * dist.k3 * r4;  // d(rf)/d(r2)

    // 畸变坐标
    double xt = 2.0 * dist.p1 * x * y + dist.p2 * (r2 + 2.0 * x * x);
    double yt = dist.p1 * (r2 + 2.0 * y * y) + 2.0 * dist.p2 * x * y;

    out_x_dist = x * rf + xt;
    out_y_dist = y * rf + yt;

    // ----- 雅可比矩阵 -----
    // d(x_dist)/dx = rf + x*drf*2x + 2*p1*y + p2*(2x+4x)
    //              = rf + 2*drf*x^2 + 2*p1*y + 6*p2*x
    Jxx = rf + 2.0 * drf * x * x + 2.0 * dist.p1 * y + 6.0 * dist.p2 * x;

    // d(x_dist)/dy = x*drf*2y + 2*p1*x + p2*2y
    //              = 2*drf*x*y + 2*p1*x + 2*p2*y
    Jxy = 2.0 * drf * x * y + 2.0 * dist.p1 * x + 2.0 * dist.p2 * y;

    // d(y_dist)/dx = y*drf*2x + p1*2x + 2*p2*y
    //              = 2*drf*x*y + 2*p1*x + 2*p2*y
    Jyx = 2.0 * drf * x * y + 2.0 * dist.p1 * x + 2.0 * dist.p2 * y;

    // d(y_dist)/dy = rf + y*drf*2y + p1*6y + 2*p2*x
    //              = rf + 2*drf*y^2 + 6*p1*y + 2*p2*x
    Jyy = rf + 2.0 * drf * y * y + 6.0 * dist.p1 * y + 2.0 * dist.p2 * x;
}

// ---------------------------------------------------------------------------
// 对单个初始猜测执行牛顿迭代（最多 100 次），
// 返回收敛后的 (x, y) 以及该点处的雅可比行列式。
// ---------------------------------------------------------------------------
static void RunNewton(
    double& x, double& y,
    double x_target, double y_target,
    const DistortionCoefficients& dist,
    double& out_det)
{
    for (int iter = 0; iter < 100; ++iter) {
        double x_curr, y_curr, Jxx, Jxy, Jyx, Jyy;
        DistortionWithJacobian(x, y, dist, x_curr, y_curr, Jxx, Jxy, Jyx, Jyy);

        double fx = x_curr - x_target;
        double fy = y_curr - y_target;

        out_det = Jxx * Jyy - Jxy * Jyx;

        if (fx * fx + fy * fy < 1e-24) return;
        if (std::abs(out_det) < 1e-30) return;

        double dx = (-fx * Jyy + fy * Jxy) / out_det;
        double dy = (-Jxx * fy + Jyx * fx) / out_det;

        // 限制步长防发散
        double step = std::sqrt(dx * dx + dy * dy);
        if (step > 5.0) {
            dx *= 5.0 / step;
            dy *= 5.0 / step;
        }

        x += dx;
        y += dy;
    }
}

// ---------------------------------------------------------------------------
// 牛顿迭代法求逆畸变：给定目标 (x_dist_target, y_dist_target)，
// 求解输入 (x_norm, y_norm) 使得畸变后恰等于目标值。
//
// 切向畸变较大时可能产生折叠映射——一个像素对应两个不同的输入
// 方向。较小的输入半径对应正确解（光线不经折叠到达该像素），
// 较大的半径是折叠回来的伪解。
//
// 本函数从多个不同缩放倍率的初始猜测出发运行牛顿迭代，
// 在满足 det(J) > 0（保持定向）的解中取半径最小的作为正确解。
// ---------------------------------------------------------------------------
static bool InverseDistortion(
    double x_target, double y_target,
    const DistortionCoefficients& dist,
    double& out_x, double& out_y)
{
    // 无畸变时直接返回
    if (dist.k1 == 0.0 && dist.k2 == 0.0 && dist.k3 == 0.0 &&
        dist.p1 == 0.0 && dist.p2 == 0.0) {
        out_x = x_target;
        out_y = y_target;
        return true;
    }

    // 用多个不同缩放因子的初始猜测来捕获不同收敛盆地
    const double scales[] = { 1.0, 2.0, 5.0, 0.5 };
    const int NUM_ATTEMPTS = 4;

    double best_x = 0.0, best_y = 0.0;
    double best_r2 = 1e30;   // 寻找最小半径（正确解）
    bool found = false;

    for (int attempt = 0; attempt < NUM_ATTEMPTS; ++attempt) {
        double x = x_target * scales[attempt];
        double y = y_target * scales[attempt];
        double det = 0.0;

        RunNewton(x, y, x_target, y_target, dist, det);

        // 跳过折叠解：当 det(J) <= 0 时映射发生了翻转，不是有效解
        if (det <= 0.0)
            continue;

        double r2 = x * x + y * y;
        if (r2 < best_r2) {
            best_r2 = r2;
            best_x = x;
            best_y = y;
            found = true;
        }
    }

    out_x = best_x;
    out_y = best_y;
    return found;
}

// ---------------------------------------------------------------------------
// 计算给定方向 φ 上雅可比行列式 det(J(r, φ)) = 0 的根（折叠半径）
//
// 沿方向 (cos(φ), sin(φ)) 从 r=0 向外搜索，当 det(J) 从正变负时
// 则找到了折叠点。若 det(J) 始终为正，返回一个很大的值（无折叠）。
// ---------------------------------------------------------------------------
static double FindFoldingRadiusInDirection(
    double cos_phi, double sin_phi,
    const DistortionCoefficients& dist)
{
    // 在 r=0 处：rf=1, drf=0
    // Jxx=1, Jyy=1, Jxy=Jyx=0 → det=1>0，初始点未折叠

    double low_r  = 0.0;
    double high_r = 1e-6;
    double high_det = 1.0;  // positive

    // 指数翻倍找到折叠区间
    for (int iter = 0; iter < 60; ++iter) {
        double x = high_r * cos_phi;
        double y = high_r * sin_phi;

        double xd, yd, Jxx, Jxy, Jyx, Jyy;
        DistortionWithJacobian(x, y, dist, xd, yd, Jxx, Jxy, Jyx, Jyy);
        high_det = Jxx * Jyy - Jxy * Jyx;

        if (high_det <= 0.0) break;

        low_r = high_r;
        high_r *= 2.0;

        if (high_r > 1e6) {
            // 没有折叠（畸变很小或为负），返回无穷大
            return 1e30;
        }
    }

    if (high_det > 0.0) {
        return 1e30;  // 未检测到折叠
    }

    // 二分法精确求解 det(J) = 0
    for (int iter = 0; iter < 80; ++iter) {
        double mid_r = (low_r + high_r) * 0.5;
        double x = mid_r * cos_phi;
        double y = mid_r * sin_phi;

        double xd, yd, Jxx, Jxy, Jyx, Jyy;
        DistortionWithJacobian(x, y, dist, xd, yd, Jxx, Jxy, Jyx, Jyy);
        double mid_det = Jxx * Jyy - Jxy * Jyx;

        if (mid_det > 0.0) {
            low_r = mid_r;
        } else {
            high_r = mid_r;
        }

        if (high_r - low_r < 1e-14) break;
    }

    return (low_r + high_r) * 0.5;
}

// ---------------------------------------------------------------------------
// 沿传感器边界采样，计算最大半视场角（考虑完整径向+切向畸变）
//
// 对每条边上的点采样，流程如下：
// 1. 确定该边界点相对主点的方向 φ
// 2. 沿该方向搜索 det(J)=0 的折叠半径 r_fold
// 3. 用牛顿法求逆畸变得到输入半径 r_newton
// 4. 取 r_effective = min(r_newton, r_fold)
// 5. 所有采样点中取 max(atan(r_effective)) 作为 FOV
//
// 这样既处理了折叠（折叠半径就是该方向的最大有效输入），
// 又保留了牛顿法在未折叠区域的精确性。
// ---------------------------------------------------------------------------
static double ComputeMaxAngleByBoundarySampling(
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion)
{
    const int SAMPLES_PER_EDGE = 100;
    double max_angle = 0.0;

    double fx = intrinsics.fx;
    double fy = intrinsics.fy;
    double cx = intrinsics.cx;
    double cy = intrinsics.cy;
    double w  = static_cast<double>(intrinsics.width - 1);
    double h  = static_cast<double>(intrinsics.height - 1);

    // 遍历四条边
    for (int edge = 0; edge < 4; ++edge) {
        for (int s = 0; s <= SAMPLES_PER_EDGE; ++s) {
            double t = static_cast<double>(s) / SAMPLES_PER_EDGE;
            double u, v;

            switch (edge) {
            case 0: u = t * w;       v = 0.0;     break; // 上边
            case 1: u = w;           v = t * h;    break; // 右边
            case 2: u = (1.0 - t) * w; v = h;      break; // 下边
            case 3: u = 0.0;         v = (1.0 - t) * h; break; // 左边
            }

            double x_dist = (u - cx) / fx;
            double y_dist = (v - cy) / fy;
            double r_dist = std::sqrt(x_dist * x_dist + y_dist * y_dist);

            if (r_dist < 1e-15) continue;  // 主点本身

            // 该边界点方向（畸变坐标系中的方向）
            double cos_phi = x_dist / r_dist;
            double sin_phi = y_dist / r_dist;

            // 找该方向的折叠半径
            double r_fold = FindFoldingRadiusInDirection(cos_phi, sin_phi, distortion);

            // 牛顿法求逆畸变
            double x_norm, y_norm;
            bool newton_ok = InverseDistortion(x_dist, y_dist, distortion, x_norm, y_norm);
            double r_newton = newton_ok ? std::sqrt(x_norm * x_norm + y_norm * y_norm) : 1e30;

            // 有效输入半径为：min(牛顿结果, 折叠半径)
            double r_eff = std::min(r_newton, r_fold);

            double angle = std::atan(r_eff);
            if (angle > max_angle) {
                max_angle = angle;
            }
        }
    }

    return max_angle;
}

// ---------------------------------------------------------------------------
// 计算最大半视场角（公开 API，带缓存）
//
// 无切向畸变时：用径向二分法（快速精确）
// 有切向畸变时：沿传感器边界密集采样，用牛顿迭代求逆完整畸变（含切向），
//               取各方向最大角度作为 FOV（保证至少一个方向可达）
// ---------------------------------------------------------------------------
double ComputeMaxHalfFovAngle(
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion)
{
    // 静态缓存：保存上一次的输入参数与计算结果
    static double cached_fx, cached_fy, cached_cx, cached_cy;
    static int    cached_width, cached_height;
    static double cached_k1, cached_k2, cached_p1, cached_p2, cached_k3;
    static double cached_result;
    static bool   cache_valid = false;

    // 检查缓存是否仍然有效
    if (cache_valid &&
        cached_fx     == intrinsics.fx  &&
        cached_fy     == intrinsics.fy  &&
        cached_cx     == intrinsics.cx  &&
        cached_cy     == intrinsics.cy  &&
        cached_width  == intrinsics.width  &&
        cached_height == intrinsics.height &&
        cached_k1     == distortion.k1 &&
        cached_k2     == distortion.k2 &&
        cached_p1     == distortion.p1 &&
        cached_p2     == distortion.p2 &&
        cached_k3     == distortion.k3)
    {
        return cached_result;
    }

    // 如果没有设置图像尺寸，默认返回 90°
    if (intrinsics.width <= 0 || intrinsics.height <= 0) {
        cached_fx = intrinsics.fx; cached_fy = intrinsics.fy;
        cached_cx = intrinsics.cx; cached_cy = intrinsics.cy;
        cached_width = intrinsics.width; cached_height = intrinsics.height;
        cached_k1 = distortion.k1; cached_k2 = distortion.k2;
        cached_p1 = distortion.p1; cached_p2 = distortion.p2;
        cached_k3 = distortion.k3;
        cached_result = M_PI / 2.0;
        cache_valid = true;
        return M_PI / 2.0;
    }

    double result;

    if (distortion.p1 == 0.0 && distortion.p2 == 0.0) {
        // ---------- 只有径向畸变：使用高效的二分法 ----------
        // 1) 传感器最远角的归一化畸变半径
        double corners_x[2] = {
            (0.0               - intrinsics.cx) / intrinsics.fx,
            (intrinsics.width  - 1.0 - intrinsics.cx) / intrinsics.fx
        };
        double corners_y[2] = {
            (0.0                - intrinsics.cy) / intrinsics.fy,
            (intrinsics.height  - 1.0 - intrinsics.cy) / intrinsics.fy
        };
        double max_r2 = 0.0;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                max_r2 = std::max(max_r2, corners_x[i]*corners_x[i] + corners_y[j]*corners_y[j]);
        double max_r_dist = std::sqrt(max_r2);

        // 2) 二分法反推输入半径
        double r = max_r_dist;
        if (distortion.k1 != 0.0 || distortion.k2 != 0.0 || distortion.k3 != 0.0) {
            // 径向畸变函数 r_dist(r) = r * (1 + k1*r^2 + k2*r^4 + k3*r^6)
            auto radial_func = [&](double rr) -> double {
                double r2 = rr * rr;
                return rr * (1.0 + distortion.k1 * r2 + distortion.k2 * r2*r2 + distortion.k3 * r2*r2*r2);
            };

            double low = 0.0, high = max_r_dist;
            if (radial_func(high) < max_r_dist) {
                while (radial_func(high) < max_r_dist && high < 1e6)
                    high *= 2.0;
            }
            for (int i = 0; i < 100; ++i) {
                double mid = (low + high) * 0.5;
                if (radial_func(mid) < max_r_dist) low = mid;
                else high = mid;
            }
            r = (low + high) * 0.5;
        }
        result = std::atan(r);

    } else {
        // ---------- 有切向畸变：沿边界采样求逆完整畸变 ----------
        result = ComputeMaxAngleByBoundarySampling(intrinsics, distortion);
    }

    // 更新缓存
    cached_fx = intrinsics.fx; cached_fy = intrinsics.fy;
    cached_cx = intrinsics.cx; cached_cy = intrinsics.cy;
    cached_width = intrinsics.width; cached_height = intrinsics.height;
    cached_k1 = distortion.k1; cached_k2 = distortion.k2;
    cached_p1 = distortion.p1; cached_p2 = distortion.p2;
    cached_k3 = distortion.k3;
    cached_result = result;
    cache_valid = true;

    return result;
}

// ---------------------------------------------------------------------------
// 计算点到相机轴线的夹角
// ---------------------------------------------------------------------------
static double ComputeAngleFromAxis(const Point3D& point)
{
    double radius = std::sqrt(point.x * point.x + point.y * point.y);
    return std::atan2(radius, point.z);
}

// 应用径向 + 切向畸变
static void ApplyDistortion(
    double x_norm,
    double y_norm,
    const DistortionCoefficients& dist,
    double& out_x_dist,
    double& out_y_dist)
{
    double r2 = x_norm * x_norm + y_norm * y_norm;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    // 径向畸变因子
    double radial_factor = 1.0 + dist.k1 * r2 + dist.k2 * r4 + dist.k3 * r6;

    // 切向畸变
    double x_tangential = 2.0 * dist.p1 * x_norm * y_norm + dist.p2 * (r2 + 2.0 * x_norm * x_norm);
    double y_tangential = dist.p1 * (r2 + 2.0 * y_norm * y_norm) + 2.0 * dist.p2 * x_norm * y_norm;

    out_x_dist = x_norm * radial_factor + x_tangential;
    out_y_dist = y_norm * radial_factor + y_tangential;
}

// 将畸变后的归一化坐标转为像素坐标
static Point2D ToPixelCoordinates(
    double x_dist,
    double y_dist,
    const CameraIntrinsics& intrinsics)
{
    Point2D pixel;
    pixel.x = intrinsics.fx * x_dist + intrinsics.cx;
    pixel.y = intrinsics.fy * y_dist + intrinsics.cy;
    return pixel;
}

Point2D ProjectPoint(
    const Point3D& point,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion)
{
    double x_norm = 0.0, y_norm = 0.0;
    bool valid = true;

    if (point.z > 0.0) {
        // 点在相机前方：正常投影
        NormalizedProjection(point, x_norm, y_norm);

        // 检查是否超出画面最大半视场角
        if (intrinsics.width > 0 && intrinsics.height > 0) {
            double max_angle = ComputeMaxHalfFovAngle(intrinsics, distortion);
            double point_angle = ComputeAngleFromAxis(point);
            if (point_angle > max_angle) {
                valid = false;
            }
        }
    } else if (point.z < 0.0) {
        // 点在相机后方：仍计算投影坐标（镜像结果），标记为无效
        NormalizedProjection(point, x_norm, y_norm);
        valid = false;
    } else {
        // z == 0，位于相机平面上：无法归一化，保持 (0,0) 标记为无效
        valid = false;
    }

    // 始终应用畸变并转换为像素坐标
    double x_dist, y_dist;
    ApplyDistortion(x_norm, y_norm, distortion, x_dist, y_dist);

    Point2D result = ToPixelCoordinates(x_dist, y_dist, intrinsics);
    result.valid = valid;
    return result;
}

Triangle2D ProjectTriangle(
    const Triangle3D& triangle,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion)
{
    Triangle2D result;
    result.v0 = ProjectPoint(triangle.v0, intrinsics, distortion);
    result.v1 = ProjectPoint(triangle.v1, intrinsics, distortion);
    result.v2 = ProjectPoint(triangle.v2, intrinsics, distortion);
    return result;
}

std::vector<Triangle2D> ProjectTriangles(
    const std::vector<Triangle3D>& triangles,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion)
{
    std::vector<Triangle2D> result;
    result.reserve(triangles.size());

    for (const auto& tri : triangles) {
        result.push_back(ProjectTriangle(tri, intrinsics, distortion));
    }

    return result;
}
