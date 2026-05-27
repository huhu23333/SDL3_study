#include <scene_node.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

// =============================================================================
// Transform3D
// =============================================================================

Point3D Transform3D::ToParent(const Point3D& local) const
{
    double dx = local.x;
    double dy = local.y;
    double dz = local.z;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double x1 = dx * cr - dy * sr;
    double y1 = dx * sr + dy * cr;
    double z1 = dz;

    double x2 = x1;
    double y2 = y1 * cp + z1 * sp;
    double z2 = -y1 * sp + z1 * cp;

    double x3 = x2 * cy + z2 * sy;
    double y3 = y2;
    double z3 = -x2 * sy + z2 * cy;

    return { x3 + tx, y3 + ty, z3 + tz };
}

Point3D Transform3D::FromParent(const Point3D& parent) const
{
    double dx = parent.x - tx;
    double dy = parent.y - ty;
    double dz = parent.z - tz;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double x1 = dx * cy - dz * sy;
    double y1 = dy;
    double z1 = dx * sy + dz * cy;

    double x2 = x1;
    double y2 = y1 * cp - z1 * sp;
    double z2 = y1 * sp + z1 * cp;

    double x3 = x2 * cr + y2 * sr;
    double y3 = -x2 * sr + y2 * cr;
    double z3 = z2;

    return { x3, y3, z3 };
}

// =============================================================================
// SceneNode
// =============================================================================

SceneNode::SceneNode(const std::string& /*name*/)
{
}

SceneNode::~SceneNode() = default;

void SceneNode::SetParent(SceneNode* parent)
{
    if (m_parent) {
        auto& siblings = m_parent->m_children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
    }
    m_parent = parent;
    if (parent) {
        parent->m_children.push_back(this);
    }
}

SceneNode* SceneNode::GetParent() const { return m_parent; }

void SceneNode::AddChild(SceneNode* child)
{
    if (child) {
        child->SetParent(this);
    }
}

const std::vector<SceneNode*>& SceneNode::GetChildren() const { return m_children; }

void SceneNode::SetLocalTransform(const Transform3D& t) { m_local = t; }
const Transform3D& SceneNode::GetLocalTransform() const { return m_local; }
Transform3D& SceneNode::GetLocalTransform() { return m_local; }

void SceneNode::SetLocalPosition(double x, double y, double z)
{
    m_local.tx = x; m_local.ty = y; m_local.tz = z;
}

void SceneNode::SetLocalRotation(double yaw, double pitch, double roll)
{
    m_local.yaw = yaw; m_local.pitch = pitch; m_local.roll = roll;
}

Point3D SceneNode::LocalToWorld(const Point3D& local_pt) const
{
    Point3D pt = local_pt;
    const SceneNode* node = this;
    while (node) {
        pt = node->m_local.ToParent(pt);
        node = node->m_parent;
    }
    return pt;
}

Point3D SceneNode::WorldToLocal(const Point3D& world_pt) const
{
    std::vector<const SceneNode*> chain;
    const SceneNode* node = this;
    while (node) {
        chain.push_back(node);
        node = node->m_parent;
    }

    Point3D pt = world_pt;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        pt = (*it)->m_local.FromParent(pt);
    }
    return pt;
}

Point3D SceneNode::ChildLocalToWorld(const Point3D& child_local, const SceneNode* child) const
{
    Point3D pt = child_local;
    const SceneNode* node = child;
    while (node) {
        pt = node->m_local.ToParent(pt);
        node = node->m_parent;
    }
    return pt;
}

// =============================================================================
// 工具函数
// =============================================================================

void BuildFaceTriangles(std::vector<WorldVertex>& out_verts,
                        double cx, double cy, double cz,
                        double width, double height,
                        float uv_l, float uv_t,
                        float uv_r, float uv_b)
{
    int segs_w = std::max(1, (int)std::ceil(width / SUBDIV_SCALE));
    int segs_h = std::max(1, (int)std::ceil(height / SUBDIV_SCALE));
    double hw = width / 2.0, hh = height / 2.0;

    out_verts.clear();
    out_verts.reserve(segs_w * segs_h * 6);

    for (int gy = 0; gy < segs_h; ++gy) {
        for (int gx = 0; gx < segs_w; ++gx) {
            double txl = (double)gx / segs_w;
            double txr = (double)(gx + 1) / segs_w;
            double tyb = (double)gy / segs_h;
            double tyt = (double)(gy + 1) / segs_h;

            double xl = cx - hw + txl * width;
            double xr = cx - hw + txr * width;
            double yb = cy - hh + tyb * height;
            double yt = cy - hh + tyt * height;

            float ul = uv_l + (uv_r - uv_l) * (float)txl;
            float ur = uv_l + (uv_r - uv_l) * (float)txr;
            float vb = uv_t + (uv_b - uv_t) * (float)tyb;
            float vt = uv_t + (uv_b - uv_t) * (float)tyt;

            out_verts.push_back({ { xl, yb, cz }, ul, vb });
            out_verts.push_back({ { xr, yb, cz }, ur, vb });
            out_verts.push_back({ { xr, yt, cz }, ur, vt });
            out_verts.push_back({ { xl, yb, cz }, ul, vb });
            out_verts.push_back({ { xr, yt, cz }, ur, vt });
            out_verts.push_back({ { xl, yt, cz }, ul, vt });
        }
    }
}

Point3D WorldToCameraTransform(const Point3D& world_pt, const Point3D& cam_pos,
                                double yaw, double pitch, double roll)
{
    double tx = world_pt.x - cam_pos.x;
    double ty = world_pt.y - cam_pos.y;
    double tz = world_pt.z - cam_pos.z;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double x1 =  tx * cy - tz * sy;
    double y1 =  ty;
    double z1 =  tx * sy + tz * cy;

    double x2 = x1;
    double y2 = y1 * cp + z1 * sp;
    double z2 = -y1 * sp + z1 * cp;

    double x3 = x2 * cr - y2 * sr;
    double y3 = x2 * sr + y2 * cr;
    double z3 = z2;

    return { x3, y3, z3 };
}

Point3D KeypointPixelToWorld(const Keypoint& kp, const TargetFaceInfo& face,
                              int tex_w, int tex_h)
{
    double u = kp.pixel_x / tex_w;
    double v = kp.pixel_y / tex_h;
    double wx = face.center_x - face.half_width  + u * face.width;
    double wy = face.center_y - face.half_height + v * face.height;
    double wz = face.center_z;
    return { wx, wy, wz };
}

void DrawFilledCircle(SDL_Renderer* renderer, float cx, float cy, float radius, SDL_FColor color)
{
    const int SEGMENTS = 24;
    std::vector<SDL_Vertex> verts;
    verts.reserve((SEGMENTS + 1) * 3);

    SDL_Vertex center = { { cx, cy }, color, { 0.0f, 0.0f } };

    for (int i = 0; i < SEGMENTS; ++i) {
        float a1 = (float)(2.0 * M_PI * i / SEGMENTS);
        float a2 = (float)(2.0 * M_PI * (i + 1) / SEGMENTS);

        float x1 = cx + radius * std::cos(a1);
        float y1 = cy + radius * std::sin(a1);
        float x2 = cx + radius * std::cos(a2);
        float y2 = cy + radius * std::sin(a2);

        verts.push_back(center);
        verts.push_back({ { x1, y1 }, color, { 0.0f, 0.0f } });
        verts.push_back({ { x2, y2 }, color, { 0.0f, 0.0f } });
    }

    SDL_RenderGeometry(renderer, nullptr, verts.data(), (int)verts.size(), nullptr, 0);
}

// =============================================================================
// ImageNode
// =============================================================================

ImageNode::ImageNode(const std::string& name)
    : SceneNode(name) {}

ImageNode::~ImageNode()
{
    if (m_texture && m_owns_texture) {
        SDL_DestroyTexture(m_texture);
    }
}

void ImageNode::SetTexture(SDL_Texture* tex, int w, int h)
{
    m_texture = tex;
    m_tex_width = w;
    m_tex_height = h;
    UpdateFaces();
}

void ImageNode::SetTextureFromMemory(SDL_Renderer* renderer,
                                      const unsigned char* pixel_data,
                                      int width, int height,
                                      SDL_PixelFormat format)
{
    if (m_texture && m_owns_texture) {
        SDL_DestroyTexture(m_texture);
    }
    m_texture = SDL_CreateTexture(renderer, format,
                                   SDL_TEXTUREACCESS_STATIC, width, height);
    if (m_texture) {
        SDL_UpdateTexture(m_texture, nullptr, pixel_data, width * 4);
        m_tex_width = width;
        m_tex_height = height;
        m_owns_texture = true;
        UpdateFaces();
    }
}

void ImageNode::SetDisplaySize(double width, double height)
{
    m_display_width = width;
    m_display_height = height;
    UpdateFaces();
}

void ImageNode::SetAlpha(float alpha) { m_alpha = alpha; }
float ImageNode::GetAlpha() const { return m_alpha; }

void ImageNode::SetBorderWidth(double w) { m_border_width = w; }
void ImageNode::SetBorderColor(SDL_FColor c) { m_border_color = c; }

const std::vector<RenderFace>& ImageNode::GetFaces() const { return m_faces; }
std::vector<RenderFace>& ImageNode::GetFaces() { return m_faces; }

SDL_Texture* ImageNode::GetTexture() const { return m_texture; }
int ImageNode::GetTexWidth() const { return m_tex_width; }
int ImageNode::GetTexHeight() const { return m_tex_height; }

void ImageNode::SetKeypoints(const std::vector<Keypoint>& kps)
{
    m_keypoints = kps;
}

const std::vector<Keypoint>& ImageNode::GetKeypoints() const
{
    return m_keypoints;
}

Point3D ImageNode::GetKeypointWorldPos(size_t index) const
{
    if (index >= m_keypoints.size()) return {0,0,0};

    const Keypoint& kp = m_keypoints[index];
    double hw = m_display_width / 2.0;
    double hh = m_display_height / 2.0;

    double u = kp.pixel_x / m_tex_width;
    double v = kp.pixel_y / m_tex_height;
    double wx = -hw + u * m_display_width;
    double wy = -hh + v * m_display_height;
    double wz = 0.0;

    Point3D local_pt = { wx, wy, wz };

    // 通过节点树级联变换到世界坐标（包含自身的局部变换和父级旋转）
    return LocalToWorld(local_pt);
}

void ImageNode::RenderKeypoints(SDL_Renderer* renderer,
                                 const CameraIntrinsics& intrinsics,
                                 const DistortionCoefficients& distortion,
                                 const Point3D& cam_pos,
                                 double cam_yaw, double cam_pitch, double cam_roll,
                                 const std::vector<std::vector<ExtraTextureInfo>>& all_extra_textures) const
{
    if (m_keypoints.empty()) return;

    SDL_FColor kp_color_dot = { 0.0f, 1.0f, 0.0f, 1.0f };
    const float MAX_COORD = 1e6f;

    for (size_t ki = 0; ki < m_keypoints.size(); ++ki) {
        Point3D world_pt = GetKeypointWorldPos(ki);

        Point3D cam_pt = WorldToCameraTransform(world_pt, cam_pos, cam_yaw, cam_pitch, cam_roll);
        if (cam_pt.z <= 0.001) continue;

        Point2D screen_pt = ProjectPoint(cam_pt, intrinsics, distortion);
        if (std::isnan(screen_pt.x) || std::isnan(screen_pt.y) ||
            std::abs(screen_pt.x) > MAX_COORD || std::abs(screen_pt.y) > MAX_COORD)
            continue;

        float sx = (float)screen_pt.x;
        float sy_float = (float)screen_pt.y;

        // 绘制关键点圆点
        DrawFilledCircle(renderer, sx, sy_float, 10.0f, kp_color_dot);

        // 绘制由外部合并传入的所有附加纹理（含序号、坐标标签等）
        if (ki < all_extra_textures.size()) {
            for (const auto& extra : all_extra_textures[ki]) {
                if (!extra.texture) continue;
                float tw, th;
                SDL_GetTextureSize(extra.texture, &tw, &th);
                SDL_FRect dst = { extra.offset_x, extra.offset_y, tw, th };
                SDL_RenderTexture(renderer, extra.texture, nullptr, &dst);
            }
        }
    }
}

void ImageNode::UpdateFaces()
{
    m_faces.clear();

    if (!m_texture) return;

    double hw = m_display_width / 2.0;
    double hh = m_display_height / 2.0;

    {
        RenderFace face;
        BuildFaceTriangles(face.world_verts,
                           0, 0, 0,
                           m_display_width, m_display_height,
                           0.0f, 0.0f, 1.0f, 1.0f);
        face.texture = m_texture;
        face.color = { 1.0f, 1.0f, 1.0f, m_alpha };
        m_faces.push_back(std::move(face));
    }

    if (m_border_width > 0.0) {
        SDL_FColor bc = m_border_color;

        {
            RenderFace face;
            BuildFaceTriangles(face.world_verts,
                               -(hw + m_border_width / 2.0), 0, 0,
                               m_border_width, m_display_height,
                               0, 0, 0, 0);
            face.texture = nullptr;
            face.color = bc;
            m_faces.push_back(std::move(face));
        }
        {
            RenderFace face;
            BuildFaceTriangles(face.world_verts,
                               (hw + m_border_width / 2.0), 0, 0,
                               m_border_width, m_display_height,
                               0, 0, 0, 0);
            face.texture = nullptr;
            face.color = bc;
            m_faces.push_back(std::move(face));
        }
        {
            RenderFace face;
            BuildFaceTriangles(face.world_verts,
                               0, (hh + m_border_width / 2.0), 0,
                               m_display_width + 2.0 * m_border_width, m_border_width,
                               0, 0, 0, 0);
            face.texture = nullptr;
            face.color = bc;
            m_faces.push_back(std::move(face));
        }
        {
            RenderFace face;
            BuildFaceTriangles(face.world_verts,
                               0, -(hh + m_border_width / 2.0), 0,
                               m_display_width + 2.0 * m_border_width, m_border_width,
                               0, 0, 0, 0);
            face.texture = nullptr;
            face.color = bc;
            m_faces.push_back(std::move(face));
        }
    }
}

void ImageNode::Render(SDL_Renderer* renderer,
                        const CameraIntrinsics& intrinsics,
                        const DistortionCoefficients& distortion,
                        const Point3D& cam_pos,
                        double cam_yaw, double cam_pitch, double cam_roll,
                        bool /*apply_body_rotation*/,
                        double /*body_rot_yaw*/,
                        double /*body_rot_pitch*/,
                        double /*body_rot_roll*/)
{
    struct SortedFace {
        const RenderFace* face;
        std::vector<SDL_Vertex> sdl_verts;
        double cam_z;
    };
    std::vector<SortedFace> sorted_faces;
    const float MAX_COORD = 1e6f;

    for (const auto& face : m_faces) {
        SortedFace sf;
        sf.face = &face;
        sf.sdl_verts.reserve(face.world_verts.size());
        double z_sum = 0.0;
        int visible_tris = 0;

        for (size_t i = 0; i < face.world_verts.size(); i += 3) {
            const WorldVertex& wv0 = face.world_verts[i];
            const WorldVertex& wv1 = face.world_verts[i + 1];
            const WorldVertex& wv2 = face.world_verts[i + 2];

            // 通过节点树级联变换到世界坐标（包含自身的局部变换和父级旋转）
            Point3D r0 = LocalToWorld(wv0.pos);
            Point3D r1 = LocalToWorld(wv1.pos);
            Point3D r2 = LocalToWorld(wv2.pos);

            Point3D c0 = WorldToCameraTransform(r0, cam_pos, cam_yaw, cam_pitch, cam_roll);
            Point3D c1 = WorldToCameraTransform(r1, cam_pos, cam_yaw, cam_pitch, cam_roll);
            Point3D c2 = WorldToCameraTransform(r2, cam_pos, cam_yaw, cam_pitch, cam_roll);

            Point2D pp0 = ProjectPoint(c0, intrinsics, distortion);
            Point2D pp1 = ProjectPoint(c1, intrinsics, distortion);
            Point2D pp2 = ProjectPoint(c2, intrinsics, distortion);

            if (!pp0.valid && !pp1.valid && !pp2.valid)
                continue;
            if (std::isnan(pp0.x) || std::isnan(pp0.y) ||
                std::isnan(pp1.x) || std::isnan(pp1.y) ||
                std::isnan(pp2.x) || std::isnan(pp2.y))
                continue;
            if (std::abs(pp0.x) > MAX_COORD || std::abs(pp0.y) > MAX_COORD ||
                std::abs(pp1.x) > MAX_COORD || std::abs(pp1.y) > MAX_COORD ||
                std::abs(pp2.x) > MAX_COORD || std::abs(pp2.y) > MAX_COORD)
                continue;

            SDL_FColor face_color = face.color;
            face_color.a = m_alpha;

            sf.sdl_verts.push_back({ { (float)pp0.x, (float)pp0.y }, face_color, { wv0.u, wv0.v } });
            sf.sdl_verts.push_back({ { (float)pp1.x, (float)pp1.y }, face_color, { wv1.u, wv1.v } });
            sf.sdl_verts.push_back({ { (float)pp2.x, (float)pp2.y }, face_color, { wv2.u, wv2.v } });
            z_sum += c0.z + c1.z + c2.z;
            visible_tris += 3;
        }

        if (visible_tris > 0) {
            sf.cam_z = z_sum / visible_tris;
            sorted_faces.push_back(std::move(sf));
        }
    }

    std::sort(sorted_faces.begin(), sorted_faces.end(),
        [](const SortedFace& a, const SortedFace& b) { return a.cam_z > b.cam_z; });

    for (const auto& sf : sorted_faces) {
        SDL_RenderGeometry(renderer, sf.face->texture,
                           sf.sdl_verts.data(), (int)sf.sdl_verts.size(),
                           nullptr, 0);
    }
}

// =============================================================================
// Scene
// =============================================================================

Scene::Scene() = default;
Scene::~Scene() = default;

SceneNode* Scene::AddNode(SceneNodePtr node)
{
    SceneNode* ptr = node.get();
    m_nodes.push_back(std::move(node));
    return ptr;
}

std::vector<SceneNode*> Scene::GetRootNodes() const
{
    std::vector<SceneNode*> roots;
    for (auto& node : m_nodes) {
        if (!node->GetParent()) {
            roots.push_back(node.get());
        }
    }
    return roots;
}

void Scene::RenderAll(SDL_Renderer* renderer,
                       const CameraIntrinsics& intrinsics,
                       const DistortionCoefficients& distortion,
                       const Point3D& cam_pos,
                       double cam_yaw, double cam_pitch, double cam_roll) const
{
    struct SortedFace {
        const RenderFace* face;
        const ImageNode* node;
        std::vector<SDL_Vertex> sdl_verts;
        double cam_z;
    };
    std::vector<SortedFace> sorted_faces;
    const float MAX_COORD = 1e6f;

    for (const auto& node : m_nodes) {
        const ImageNode* img_node = dynamic_cast<const ImageNode*>(node.get());
        if (!img_node) continue;

        for (const auto& face : img_node->GetFaces()) {
            SortedFace sf;
            sf.face = &face;
            sf.node = img_node;
            sf.sdl_verts.reserve(face.world_verts.size());
            double z_sum = 0.0;
            int visible_tris = 0;

            for (size_t i = 0; i < face.world_verts.size(); i += 3) {
                const WorldVertex& wv0 = face.world_verts[i];
                const WorldVertex& wv1 = face.world_verts[i + 1];
                const WorldVertex& wv2 = face.world_verts[i + 2];

                // 通过节点树级联变换到世界坐标（包含自身的局部变换和父级旋转）
                Point3D r0 = img_node->LocalToWorld(wv0.pos);
                Point3D r1 = img_node->LocalToWorld(wv1.pos);
                Point3D r2 = img_node->LocalToWorld(wv2.pos);

                Point3D c0 = WorldToCameraTransform(r0, cam_pos, cam_yaw, cam_pitch, cam_roll);
                Point3D c1 = WorldToCameraTransform(r1, cam_pos, cam_yaw, cam_pitch, cam_roll);
                Point3D c2 = WorldToCameraTransform(r2, cam_pos, cam_yaw, cam_pitch, cam_roll);

                Point2D pp0 = ProjectPoint(c0, intrinsics, distortion);
                Point2D pp1 = ProjectPoint(c1, intrinsics, distortion);
                Point2D pp2 = ProjectPoint(c2, intrinsics, distortion);

                if (!pp0.valid && !pp1.valid && !pp2.valid)
                    continue;
                if (std::isnan(pp0.x) || std::isnan(pp0.y) ||
                    std::isnan(pp1.x) || std::isnan(pp1.y) ||
                    std::isnan(pp2.x) || std::isnan(pp2.y))
                    continue;
                if (std::abs(pp0.x) > MAX_COORD || std::abs(pp0.y) > MAX_COORD ||
                    std::abs(pp1.x) > MAX_COORD || std::abs(pp1.y) > MAX_COORD ||
                    std::abs(pp2.x) > MAX_COORD || std::abs(pp2.y) > MAX_COORD)
                    continue;

                SDL_FColor face_color = face.color;
                face_color.a = img_node->GetAlpha();

                sf.sdl_verts.push_back({ { (float)pp0.x, (float)pp0.y }, face_color, { wv0.u, wv0.v } });
                sf.sdl_verts.push_back({ { (float)pp1.x, (float)pp1.y }, face_color, { wv1.u, wv1.v } });
                sf.sdl_verts.push_back({ { (float)pp2.x, (float)pp2.y }, face_color, { wv2.u, wv2.v } });
                z_sum += c0.z + c1.z + c2.z;
                visible_tris += 3;
            }

            if (visible_tris > 0) {
                sf.cam_z = z_sum / visible_tris;
                sorted_faces.push_back(std::move(sf));
            }
        }
    }

    std::sort(sorted_faces.begin(), sorted_faces.end(),
        [](const SortedFace& a, const SortedFace& b) { return a.cam_z > b.cam_z; });

    for (const auto& sf : sorted_faces) {
        SDL_RenderGeometry(renderer, sf.face->texture,
                           sf.sdl_verts.data(), (int)sf.sdl_verts.size(),
                           nullptr, 0);
    }
}

const std::vector<SceneNodePtr>& Scene::GetAllNodes() const { return m_nodes; }

