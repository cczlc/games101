// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

std::vector<Eigen::Matrix4f> get_model_matrix(std::vector<float>& rotation_angle)
{
    //Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> model;

    Eigen::Matrix4f modelTemp;

    for(auto& angle : rotation_angle){
        float theta = angle / 180.0 * MY_PI;
        modelTemp << cos(theta), -sin(theta), 0, 0,
            sin(theta), cos(theta), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        model.push_back(modelTemp);
    }

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection;

    zNear = -zNear;
    zFar = -zFar;

    Eigen::Matrix4f squeeze;
    squeeze << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -(zNear * zFar),
        0, 0, 1, 0;

    // 定义透视投影平面大小
    float halfOfTheta = eye_fov / 2 / 180.0 * MY_PI;
    float t = -zNear * tan(halfOfTheta);
    float b = zNear * tan(halfOfTheta);
    float l = -t * aspect_ratio;
    float r = t * aspect_ratio;

    // 正交投影矩阵
    // 并将中心移动到原点
    Eigen::Matrix4f ortho;
    ortho << 2 / (r - l), 0, 0, -(r + l) / 2,
        0, 2 / (t - b), 0, -(t + b) / 2,
        0, 0, 2 / (zFar - zNear), -(zFar + zNear) / 2,            // 这个值为什么是负的？？？
        0, 0, 0, 1;

    projection = ortho * squeeze;

    return projection;
}

int main(int argc, const char** argv)
{
    std::vector<float> angle;
    // 两个三角形则初始化两个角度
    angle.push_back(0.0);
    angle.push_back(0.0);

    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    // 相机位置
    Eigen::Vector3f eye_pos = {0,0,5};


    // 两个三角形的6个点
    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    // 6个点对应的索引
    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    // 6个点对应的颜色RGB值
    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);  // 0
    auto ind_id = r.load_indices(ind);    // 1
    auto col_id = r.load_colors(cols);    // 2

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle[0] += 10;
        }
        else if (key == 'd')
        {
            angle[0] -= 10;
        }
        else if (key == 'j')
        {
            angle[1] += 10;
        }
        else if (key == 'l')
        {
            angle[1] -= 10;
        }

        //std::cout << angle[0] << " " << angle[1] << std::endl;
    }

    return 0;
}
// clang-format on