// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v0 = _v[0];
    Vector3f v1 = _v[1];
    Vector3f v2 = _v[2];
    Vector3f p = Vector3f(x, y, 1);
    Vector3f v0p = p - v0;
    Vector3f v1p = p - v1;
    Vector3f v2p = p - v2;
    if (v0p.cross(v1p).z() >= 0 && v1p.cross(v2p).z() >= 0 && v2p.cross(v0p).z() >= 0) {
        return true;
    }else {
        return false;
    }
}



static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    auto v = t.toVector4();
    int x_min = std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x())));
    int x_max = std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x())));
    int y_min = std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y())));
    int y_max = std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y())));
    
    // 将包围盒限制在屏幕范围内
    x_min = std::max(0, x_min);
    x_max = std::min(width - 1, x_max);
    y_min = std::max(0, y_min);
    y_max = std::min(height - 1, y_max);
    
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            bool is_msaa = true;
            Vector2f x4[4] = {{0, 0},
                    {0, 0},
                    {0, 0},
                    {0, 0}};
            if (is_msaa){
                x4[0] = {0.25, 0.25};
                x4[1] = {0.25, 0.75};
                x4[2] = {0.75, 0.25};
                x4[3] = {0.75, 0.75};
            }
            
            bool depth_test = false;
            for (int k = 0; k < 4; k++){
                if (!insideTriangle(x + x4[k].x(), y + x4[k].y(), t.v))
                    continue;
                auto [alpha, beta, gamma] = computeBarycentric2D(x + x4[k].x(), y + x4[k].y(), t.v);
                float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated =
                    alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                int index = get_msaa_index(x, y, k);
                if (z_interpolated < msaa_depth_buf[index]){
                    msaa_depth_buf[index] = z_interpolated;
                    msaa_color_buf[index] = t.getColor();
                    depth_test = true;
                }
            }
            if (depth_test){
                set_pixel(Vector3f(x, y, 0), get_sample_color(x, y));
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(msaa_color_buf.begin(), msaa_color_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_depth_buf.resize(w * h * 4);
    msaa_color_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_msaa_index(int x, int y, int k)
{
    return (height - 1- y) * width * 4 + (width - 1 - x) * 4 + k; //https://zhuanlan.zhihu.com/p/598823667
}

Vector3f rst::rasterizer::get_sample_color(int x, int y)
{
    Vector3f sum{0, 0, 0};
    for (int i = 0; i < 4; i++)
        sum += msaa_color_buf[get_msaa_index(x, y, i)];
    return sum / 4.0f;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on