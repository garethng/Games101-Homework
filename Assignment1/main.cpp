#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation_matrix(float rotation_angle, Eigen::Vector3f axis)
{
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f first = std::cos(rotation_angle/180*MY_PI) * Eigen::Matrix3f::Identity();
    
    Eigen::Matrix3f second;
    second = axis * axis.transpose();
    second *= (1 - std::cos(rotation_angle/180*MY_PI));
    
    Eigen::Matrix3f third;
    third << 0, -axis[2], axis[1],
            axis[2], 0, -axis[0], 
            -axis[1], axis[0], 0;
    third *= std::sin(rotation_angle/180*MY_PI);
    
    Eigen::Matrix3f rotation_3d = Eigen::Matrix3f::Identity();
    rotation_3d = first + second + third;
    rotate.block<3, 3>(0, 0) = rotation_3d;
    rotate.row(3) << 0, 0, 1, 0;
    return rotate;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Eigen::Matrix4f rotate;
    // rotate << std::cos(rotation_angle/180*MY_PI), -std::sin(rotation_angle/180*MY_PI), 0, 0,
    // std::sin(rotation_angle/180*MY_PI), std::cos(rotation_angle/180*MY_PI), 0, 0,
    // 0, 0, 1, 0,
    // 0, 0, 0, 1;
    // model = rotate * model;

    Eigen::Matrix4f model = get_rotation_matrix(rotation_angle, Eigen::Vector3f(0, 0, 1));
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float tan_half_fov = std::tan(eye_fov /180*MY_PI / 2);
    
    float t = std::abs(zNear) * tan_half_fov;
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    float n = zNear;
    float f = zFar;

    float A = zNear + zFar;
    float B = -zNear * zFar;
    Eigen::Matrix4f persp2ortho;
    persp2ortho << zNear, 0, 0, 0,
    0, zNear, 0, 0,
    0, 0, A, B,
    0, 0, 1, 0;

    Eigen::Matrix4f ortho1;

    ortho1 << 2 / (r - l), 0, 0, 0,
    0, 2 / (t - b), 0, 0,
    0, 0, 2 / (n - f), 0,
    0, 0, 0, 1;

    Eigen::Matrix4f ortho2;
    ortho2 << 1, 0, 0, -(r + l) / 2,
    0, 1, 0, -(t + b) / 2,
    0, 0, 1, -(n + f) / 2,
    0, 0, 0, 1;

    Eigen::Matrix4f ortho = ortho1 * ortho2;

    projection = ortho * persp2ortho;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
