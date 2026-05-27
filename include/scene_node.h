#ifndef SCENE_NODE_H
#define SCENE_NODE_H

#include <camera_projection.h>
#include <SDL3/SDL.h>

#include <vector>
#include <memory>
#include <string>

// -----------------------------------------------------------------------------
// 附加纹理信息（用于关键点渲染时在外部管理的附加纹理）
// -----------------------------------------------------------------------------
struct ExtraTextureInfo
{
    SDL_Texture* texture = nullptr;  // 预渲染的纹理
    float offset_x = 0.0f;           // 相对于关键点中心的 X 偏移（屏幕像素）
    float offset_y = 0.0f;           // 相对于关键点中心的 Y 偏移（屏幕像素）
};

// -----------------------------------------------------------------------------
// 三维变换：位移 + 旋转（内禀 ZYX 顺序）
// -----------------------------------------------------------------------------
struct Transform3D
{
    double tx = 0.0, ty = 0.0, tz = 0.0;
    double yaw   = 0.0;
    double pitch = 0.0;
    double roll  = 0.0;

    Point3D ToParent(const Point3D& local) const;
    Point3D FromParent(const Point3D& parent) const;
};

// -----------------------------------------------------------------------------
// 场景节点基类
// -----------------------------------------------------------------------------
class SceneNode;
using SceneNodePtr = std::shared_ptr<SceneNode>;

class SceneNode
{
public:
    SceneNode(const std::string& name = "Node");
    virtual ~SceneNode();

    void SetParent(SceneNode* parent);
    SceneNode* GetParent() const;
    void AddChild(SceneNode* child);
    const std::vector<SceneNode*>& GetChildren() const;

    void SetLocalTransform(const Transform3D& t);
    const Transform3D& GetLocalTransform() const;
    Transform3D& GetLocalTransform();

    void SetLocalPosition(double x, double y, double z);
    void SetLocalRotation(double yaw, double pitch, double roll);

    Point3D LocalToWorld(const Point3D& local_pt) const;
    Point3D WorldToLocal(const Point3D& world_pt) const;
    Point3D ChildLocalToWorld(const Point3D& child_local, const SceneNode* child) const;

    virtual void Render(SDL_Renderer* renderer,
                        const CameraIntrinsics& intrinsics,
                        const DistortionCoefficients& distortion,
                        const Point3D& cam_pos,
                        double cam_yaw, double cam_pitch, double cam_roll,
                        bool apply_body_rotation = false,
                        double body_rot_yaw = 0.0,
                        double body_rot_pitch = 0.0,
                        double body_rot_roll = 0.0) {}

protected:
    SceneNode* m_parent = nullptr;
    std::vector<SceneNode*> m_children;
    Transform3D m_local;
};

// -----------------------------------------------------------------------------
// 结构体声明
// -----------------------------------------------------------------------------
struct WorldVertex {
    Point3D pos;
    float u, v;
};

struct RenderFace {
    std::vector<WorldVertex> world_verts;
    SDL_Texture* texture = nullptr;
    SDL_FColor color = { 1.0f, 1.0f, 1.0f, 1.0f };
};

const double SUBDIV_SCALE = 0.05;

// 工具函数
void BuildFaceTriangles(std::vector<WorldVertex>& out_verts,
                        double cx, double cy, double cz,
                        double width, double height,
                        float uv_l, float uv_t,
                        float uv_r, float uv_b);

Point3D WorldToCameraTransform(const Point3D& world_pt, const Point3D& cam_pos,
                                double yaw, double pitch, double roll);

void DrawFilledCircle(SDL_Renderer* renderer, float cx, float cy, float radius, SDL_FColor color);

// -----------------------------------------------------------------------------
// 关键点
// -----------------------------------------------------------------------------
struct Keypoint
{
    int index;
    double pixel_x;
    double pixel_y;
};

struct TargetFaceInfo {
    double center_x, center_y, center_z;
    double half_width, half_height;
    double width, height;
};

Point3D KeypointPixelToWorld(const Keypoint& kp, const TargetFaceInfo& face,
                              int tex_w, int tex_h);

// -----------------------------------------------------------------------------
// 图像节点类
// -----------------------------------------------------------------------------
class ImageNode : public SceneNode
{
public:
    ImageNode(const std::string& name = "Image");
    ~ImageNode() override;

    void SetTexture(SDL_Texture* tex, int w, int h);
    void SetTextureFromMemory(SDL_Renderer* renderer,
                               const unsigned char* pixel_data,
                               int width, int height,
                               SDL_PixelFormat format = SDL_PIXELFORMAT_RGBA32);

    void SetDisplaySize(double width, double height);

    void SetAlpha(float alpha);
    float GetAlpha() const;

    void SetBorderWidth(double w);
    void SetBorderColor(SDL_FColor c);

    const std::vector<RenderFace>& GetFaces() const;
    std::vector<RenderFace>& GetFaces();

    SDL_Texture* GetTexture() const;
    int GetTexWidth() const;
    int GetTexHeight() const;

    void SetKeypoints(const std::vector<Keypoint>& kps);
    const std::vector<Keypoint>& GetKeypoints() const;
    Point3D GetKeypointWorldPos(size_t index) const;

    void RenderKeypoints(SDL_Renderer* renderer,
                          const CameraIntrinsics& intrinsics,
                          const DistortionCoefficients& distortion,
                          const Point3D& cam_pos,
                          double cam_yaw, double cam_pitch, double cam_roll,
                          const std::vector<std::vector<ExtraTextureInfo>>& all_extra_textures) const;

    void Render(SDL_Renderer* renderer,
                const CameraIntrinsics& intrinsics,
                const DistortionCoefficients& distortion,
                const Point3D& cam_pos,
                double cam_yaw, double cam_pitch, double cam_roll,
                bool apply_body_rotation = false,
                double body_rot_yaw = 0.0,
                double body_rot_pitch = 0.0,
                double body_rot_roll = 0.0) override;

protected:
    void UpdateFaces();

private:
    SDL_Texture* m_texture = nullptr;
    bool m_owns_texture = false;
    int m_tex_width = 0;
    int m_tex_height = 0;
    double m_display_width = 2.0;
    double m_display_height = 2.0;
    float m_alpha = 1.0f;
    double m_border_width = 0.0;
    SDL_FColor m_border_color = { 1.0f, 1.0f, 0.39f, 1.0f };
    std::vector<RenderFace> m_faces;
    std::vector<Keypoint> m_keypoints;
};

// -----------------------------------------------------------------------------
// 场景管理类
// -----------------------------------------------------------------------------
class Scene
{
public:
    Scene();
    ~Scene();

    SceneNode* AddNode(SceneNodePtr node);
    std::vector<SceneNode*> GetRootNodes() const;

    void RenderAll(SDL_Renderer* renderer,
                   const CameraIntrinsics& intrinsics,
                   const DistortionCoefficients& distortion,
                   const Point3D& cam_pos,
                   double cam_yaw, double cam_pitch, double cam_roll) const;

    const std::vector<SceneNodePtr>& GetAllNodes() const;

private:
    std::vector<SceneNodePtr> m_nodes;
};

#endif // SCENE_NODE_H
