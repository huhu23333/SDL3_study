#ifndef CAMERA_PROJECTION_H
#define CAMERA_PROJECTION_H

#include <vector>

// 三维空间中的点
struct Point3D
{
    double x, y, z;
};

// 二维屏幕（图像）上的点
struct Point2D
{
    double x, y;
    bool   valid = true; // 投影是否有效（false 表示在相机后方或超出视场角）
};

// 三维空间中的三角面（三个顶点）
struct Triangle3D
{
    Point3D v0, v1, v2;
};

// 投影到二维屏幕后的三角面
struct Triangle2D
{
    Point2D v0, v1, v2;
};

// 相机内参矩阵参数
//     [ fx  0  cx ]
// K = [  0  fy cy ]
//     [  0   0  1 ]
struct CameraIntrinsics
{
    double fx;      // x 方向焦距（像素单位）
    double fy;      // y 方向焦距（像素单位）
    double cx;      // 主点 x 坐标
    double cy;      // 主点 y 坐标
    int width  = 0; // 图像宽度（像素），用于计算最大半视场角；若为 0 则跳过角度检查
    int height = 0; // 图像高度（像素），用于计算最大半视场角；若为 0 则跳过角度检查
};

// 畸变系数（OpenCV 模型）
// 径向畸变: k1, k2, k3
// 切向畸变: p1, p2
struct DistortionCoefficients
{
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
};

/**
 * @brief 根据相机内参和畸变系数计算画面最大半视场角
 *
 * 考虑了径向畸变（k1, k2, k3）的影响，通过求解畸变方程
 * 计算传感器边缘所能对应的最大入射角。
 * 若图像宽高未设置（width == 0 || height == 0），则返回 π/2（90°）。
 *
 * @param intrinsics   相机内参（需包含 width 和 height）
 * @param distortion   畸变系数
 * @return double      最大半视场角（弧度），范围 (0, π/2]
 */
double ComputeMaxHalfFovAngle(
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion = DistortionCoefficients{});

/**
 * @brief 将单个三维点投影到二维屏幕坐标
 *
 * 使用针孔相机模型，先进行针孔投影，再应用径向+切向畸变。
 * 假设输入的三维点已经位于相机坐标系中（即已经过 view 变换）。
 *
 * 当 CameraIntrinsics 中设置了 width 和 height 时，会先计算画面的
 * 最大半视场角（含畸变影响），若点与相机轴线的夹角超过该角度，
 * 则将返回值的 valid 字段置为 false。
 *
 * 注意：无论点是否有效（z ≤ 0、超出视场角），始终计算投影坐标；
 * 有效性信息通过 Point2D::valid 字段表达。
 *
 * @param point        相机坐标系下的三维点 (X, Y, Z)
 * @param intrinsics   相机内参 (fx, fy, cx, cy, width, height)
 * @param distortion   畸变系数 (k1, k2, p1, p2, k3)
 * @return Point2D     屏幕像素坐标 (u, v)，valid 字段指示投影是否有效
 */
Point2D ProjectPoint(
    const Point3D& point,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion = DistortionCoefficients{});

/**
 * @brief 将三维三角面投影到二维屏幕
 *
 * @param triangle      相机坐标系下的三维三角面
 * @param intrinsics    相机内参
 * @param distortion    畸变系数
 * @return Triangle2D   投影后的二维屏幕三角面
 */
Triangle2D ProjectTriangle(
    const Triangle3D& triangle,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion = DistortionCoefficients{});

/**
 * @brief 将多个三维三角面批量投影到二维屏幕
 *
 * @param triangles     相机坐标系下的三维三角面列表
 * @param intrinsics    相机内参
 * @param distortion    畸变系数
 * @return std::vector<Triangle2D> 投影后的二维屏幕三角面列表
 */
std::vector<Triangle2D> ProjectTriangles(
    const std::vector<Triangle3D>& triangles,
    const CameraIntrinsics& intrinsics,
    const DistortionCoefficients& distortion = DistortionCoefficients{});

#endif  // CAMERA_PROJECTION_H
