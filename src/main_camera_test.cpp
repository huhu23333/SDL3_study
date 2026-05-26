#include <camera_projection.h>

#include <cstdio>
#include <cmath>
#include <vector>

// -----------------------------------------------------------------------------
// 辅助：打印一个二维点
// -----------------------------------------------------------------------------
static void PrintPoint2D(const char* label, const Point2D& pt)
{
    if (!pt.valid) {
        std::printf("  %s: (INVALID) (%.2f, %.2f)\n", label, pt.x, pt.y);
    } else {
        std::printf("  %s: (%.2f, %.2f)\n", label, pt.x, pt.y);
    }
}

// -----------------------------------------------------------------------------
// 辅助：打印一个三角面
// -----------------------------------------------------------------------------
static void PrintTriangle2D(const char* label, const Triangle2D& tri)
{
    std::printf("%s:\n", label);
    PrintPoint2D("v0", tri.v0);
    PrintPoint2D("v1", tri.v1);
    PrintPoint2D("v2", tri.v2);
}

// -----------------------------------------------------------------------------
// 辅助：比较两个浮点数是否近似相等
// -----------------------------------------------------------------------------
static bool ApproxEqual(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) < eps;
}

// -----------------------------------------------------------------------------
// 辅助：检查 Point2D 是否近似相等
// -----------------------------------------------------------------------------
static bool Point2DApproxEqual(const Point2D& a, const Point2D& b, double eps = 1e-6)
{
    return ApproxEqual(a.x, b.x, eps) && ApproxEqual(a.y, b.y, eps);
}

// -----------------------------------------------------------------------------
// 测试用例 1：理想针孔相机（无畸变）
// 中心点 (0, 0, 10) 应投影到主点 (cx, cy)
// -----------------------------------------------------------------------------
static int TestIdealPinhole()
{
    std::printf("========== Test 1: Ideal Pinhole Camera (No Distortion) ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };
    DistortionCoefficients no_distortion{};  // 所有系数默认为 0

    // 一个位于相机正前方的点，应投影到主点
    Point3D center_point{ 0.0, 0.0, 10.0 };
    Point2D center_proj = ProjectPoint(center_point, intrinsics, no_distortion);
    std::printf("Center point (0,0,10) -> ");
    PrintPoint2D("", center_proj);

    if (!Point2DApproxEqual(center_proj, Point2D{ 320.0, 240.0 })) {
        std::printf("  FAIL: Expected (320.00, 240.00)\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    // 一个位于 (1, 0, 10) 的点，x_norm = 0.1, u = 500 * 0.1 + 320 = 370
    Point3D right_point{ 1.0, 0.0, 10.0 };
    Point2D right_proj = ProjectPoint(right_point, intrinsics, no_distortion);
    std::printf("Right point (1,0,10) -> ");
    PrintPoint2D("", right_proj);
    if (!Point2DApproxEqual(right_proj, Point2D{ 370.0, 240.0 })) {
        std::printf("  FAIL: Expected (370.00, 240.00)\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 2：径向畸变（桶形畸变 / 枕形畸变）
// -----------------------------------------------------------------------------
static int TestRadialDistortion()
{
    std::printf("========== Test 2: Radial Distortion ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };

    // 枕形畸变（pincushion）：k1 > 0，边缘点被拉向更外
    DistortionCoefficients pincushion{ 0.1, 0.0, 0.0, 0.0, 0.0 };

    Point3D corner_point{ 2.0, 1.5, 10.0 };  // 在画面角落附近的点
    Point2D no_dist_proj = ProjectPoint(corner_point, intrinsics, DistortionCoefficients{});
    Point2D pincushion_proj = ProjectPoint(corner_point, intrinsics, pincushion);

    std::printf("Corner point (2.0, 1.5, 10.0):\n");
    PrintPoint2D("  No distortion", no_dist_proj);
    PrintPoint2D("  Pincushion (k1=0.1)", pincushion_proj);

    // 枕形畸变应使投影点离主点更远
    double no_dist_radius = std::hypot(no_dist_proj.x - intrinsics.cx, no_dist_proj.y - intrinsics.cy);
    double dist_radius = std::hypot(pincushion_proj.x - intrinsics.cx, pincushion_proj.y - intrinsics.cy);
    std::printf("  Radius from center: no_dist=%.2f, distorted=%.2f\n", no_dist_radius, dist_radius);

    if (dist_radius <= no_dist_radius) {
        std::printf("  FAIL: Pincushion distortion should increase radius\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    // 桶形畸变（barrel）：k1 < 0，边缘点被拉向中心
    DistortionCoefficients barrel{ -0.1, 0.0, 0.0, 0.0, 0.0 };
    Point2D barrel_proj = ProjectPoint(corner_point, intrinsics, barrel);

    PrintPoint2D("  Barrel (k1=-0.1)", barrel_proj);
    double barrel_radius = std::hypot(barrel_proj.x - intrinsics.cx, barrel_proj.y - intrinsics.cy);

    if (barrel_radius >= no_dist_radius) {
        std::printf("  FAIL: Barrel distortion should decrease radius\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 3：切向畸变
// -----------------------------------------------------------------------------
static int TestTangentialDistortion()
{
    std::printf("========== Test 3: Tangential Distortion ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };
    DistortionCoefficients tangential{ 0.0, 0.0, 0.02, 0.01, 0.0 };

    Point3D point{ 1.0, 0.5, 10.0 };
    Point2D no_dist_proj = ProjectPoint(point, intrinsics, DistortionCoefficients{});
    Point2D tan_proj = ProjectPoint(point, intrinsics, tangential);

    std::printf("Point (1.0, 0.5, 10.0):\n");
    PrintPoint2D("  No distortion", no_dist_proj);
    PrintPoint2D("  With tangential (p1=0.02, p2=0.01)", tan_proj);

    // 有畸变时投影坐标应不同
    if (Point2DApproxEqual(no_dist_proj, tan_proj, 1e-4)) {
        std::printf("  FAIL: Tangential distortion should change projection result\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 4：相机后方点应标记为 invalid
// -----------------------------------------------------------------------------
static int TestPointBehindCamera()
{
    std::printf("========== Test 4: Point Behind Camera ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };

    // Z 为负，在相机后方
    Point3D behind_point{ 0.0, 0.0, -5.0 };
    Point2D proj = ProjectPoint(behind_point, intrinsics);

    PrintPoint2D("Point (0, 0, -5)", proj);

    // 应标记为无效，但坐标仍被计算（镜像到主点）
    if (proj.valid) {
        std::printf("  FAIL: Point behind camera should have valid==false\n");
        return 1;
    }
    std::printf("  Coordinates still computed: (%.2f, %.2f)\n", proj.x, proj.y);
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 5：不同相机内参（模拟不同相机）
// -----------------------------------------------------------------------------
static int TestDifferentIntrinsics()
{
    std::printf("========== Test 5: Different Camera Intrinsics ==========\n");

    // 模拟一台广角相机（较大 FOV => 较小焦距）
    CameraIntrinsics wide_angle{ 300.0, 300.0, 320.0, 240.0 };
    // 模拟一台长焦相机（较小 FOV => 较大焦距）
    CameraIntrinsics telephoto{ 1000.0, 1000.0, 320.0, 240.0 };

    Point3D point{ 1.0, 1.0, 5.0 };
    Point2D wide_proj = ProjectPoint(point, wide_angle);
    Point2D tele_proj = ProjectPoint(point, telephoto);

    PrintPoint2D("Wide-angle (fx=fy=300)", wide_proj);
    PrintPoint2D("Telephoto  (fx=fy=1000)", tele_proj);

    // 长焦相机的投影点应比广角相机离主点更远（放大效果）
    double wide_radius = std::hypot(wide_proj.x - wide_angle.cx, wide_proj.y - wide_angle.cy);
    double tele_radius = std::hypot(tele_proj.x - telephoto.cx, tele_proj.y - telephoto.cy);

    std::printf("  Radius from center: wide=%.2f, tele=%.2f\n", wide_radius, tele_radius);

    if (tele_radius <= wide_radius) {
        std::printf("  FAIL: Telephoto should magnify more (larger radius)\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 6：批量投影多个三角面
// -----------------------------------------------------------------------------
static int TestBatchProjection()
{
    std::printf("========== Test 6: Batch Projection of Multiple Triangles ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };

    // 构造两个三角面：一个在左，一个在右
    std::vector<Triangle3D> triangles = {
        // 左三角面：中心偏左
        Triangle3D{
            Point3D{ -1.0,  0.5, 5.0 },
            Point3D{ -1.0, -0.5, 5.0 },
            Point3D{ -0.3,  0.0, 5.0 }
        },
        // 右三角面：中心偏右
        Triangle3D{
            Point3D{  0.3,  0.5, 5.0 },
            Point3D{  0.3, -0.5, 5.0 },
            Point3D{  1.0,  0.0, 5.0 }
        }
    };

    std::vector<Triangle2D> projected = ProjectTriangles(triangles, intrinsics);

    std::printf("Projected %zu triangles:\n", projected.size());
    PrintTriangle2D("  Triangle 0 (left)", projected[0]);
    PrintTriangle2D("  Triangle 1 (right)", projected[1]);

    // 验证：左三角面的所有顶点 x 应小于主点 x
    if (projected[0].v0.x >= intrinsics.cx &&
        projected[0].v1.x >= intrinsics.cx &&
        projected[0].v2.x >= intrinsics.cx) {
        std::printf("  FAIL: Left triangle should have x < cx\n");
        return 1;
    }

    // 验证：右三角面的所有顶点 x 应大于主点 x
    if (projected[1].v0.x <= intrinsics.cx &&
        projected[1].v1.x <= intrinsics.cx &&
        projected[1].v2.x <= intrinsics.cx) {
        std::printf("  FAIL: Right triangle should have x > cx\n");
        return 1;
    }

    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 测试用例 7：Z 非常小的情况（近处点）
// -----------------------------------------------------------------------------
static int TestClosePoint()
{
    std::printf("========== Test 7: Very Close Point ==========\n");

    CameraIntrinsics intrinsics{ 500.0, 500.0, 320.0, 240.0 };

    // 非常近的点，Z = 0.5
    Point3D close_point{ 0.1, 0.1, 0.5 };
    Point2D proj = ProjectPoint(close_point, intrinsics);

    PrintPoint2D("Close point (0.1, 0.1, 0.5)", proj);

    // u = 500 * (0.1/0.5) + 320 = 500 * 0.2 + 320 = 420
    // v = 500 * (0.1/0.5) + 240 = 500 * 0.2 + 240 = 340
    if (!Point2DApproxEqual(proj, Point2D{ 420.0, 340.0 })) {
        std::printf("  FAIL: Expected (420.00, 340.00)\n");
        return 1;
    }
    std::printf("  PASS\n\n");

    return 0;
}

// -----------------------------------------------------------------------------
// 主函数
// -----------------------------------------------------------------------------
int main()
{
    std::printf("============================================================\n");
    std::printf("  Camera Projection Function Test Suite\n");
    std::printf("============================================================\n\n");

    int total_failures = 0;
    total_failures += TestIdealPinhole();
    total_failures += TestRadialDistortion();
    total_failures += TestTangentialDistortion();
    total_failures += TestPointBehindCamera();
    total_failures += TestDifferentIntrinsics();
    total_failures += TestBatchProjection();
    total_failures += TestClosePoint();

    std::printf("============================================================\n");
    if (total_failures == 0) {
        std::printf("  All tests PASSED!\n");
    } else {
        std::printf("  %d test(s) FAILED!\n", total_failures);
    }
    std::printf("============================================================\n");

    return total_failures;
}
