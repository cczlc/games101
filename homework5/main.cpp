#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

int factorial(int num)
{
    if(num < 0){
        return -1;
    }

    if(num == 0)
    {
        return 1;
    }
    else
    {
        return num * factorial(num - 1);
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
 
    cv::Point2f point = {0.0f, 0.0f};
    int n = control_points.size();

    for(unsigned i = 0; i < n; ++i)
    {
        int c = factorial(n - 1) / (factorial(i) * factorial(n - 1 - i));
        point += c * std::pow(t, i) * std::pow(1 - t, n - 1 - i) * control_points[i];
    }


    return point;

    // if(control_points.size() == 2)
    // {
    //     return (1.0f - t) * control_points[0] + t * control_points[1];
    // }

    // std::vector<cv::Point2f> vec;
    // for(int i = 0; i < control_points.size() - 1; ++i)
    // {
    //     vec.push_back((1.0f - t) * control_points[i] + t * control_points[i + 1]);
    // }

    // return recursive_bezier(vec, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    
    for(double t = 0; t <= 1.0; t += 0.001 )
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        //std::cout << point.x << " " << point.y << std::endl;
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        
        // 取道距离采样点最近的4个点
        float x = point.x - floor(point.x);
        float y = point.y - floor(point.y);
        float x_flag = x < 0.5f ? -1.0f : 1.0f;
        float y_flag = y < 0.5f ? -1.0f : 1.0f;

        cv::Point2f p00 = {floor(point.x) + 0.5f, floor(point.y) + 0.5f};
        cv::Point2f p01 = {floor(point.x), floor(point.y) + y_flag + 0.5f};
        cv::Point2f p10 = {floor(point.x) + x_flag + 0.5f, floor(point.y)};
        cv::Point2f p11 = {floor(point.x) + x_flag + 0.5f, floor(point.y) + y_flag + 0.5f};

        std::vector<cv::Point2f> line_point;
        line_point.push_back(p01);
        line_point.push_back(p10);
        line_point.push_back(p11);

        // 计算最近像素点与曲线上的点的距离
        cv::Point2f temp = point - p00;
        float length =  sqrt(temp.dot(temp));


        // 令计算出的曲线的点为绿色，计算像素中点与曲线上的点的距离并填充颜色
        for(auto p : line_point)
        {
            temp = point - p;
            float len = sqrt(temp.dot(temp));
            float percent = length / len;
            //std::cout << percent << " ";

            window.at<cv::Vec3b>(p.y, p.x)[1] = (double)200 + (255 - 200) * percent;
        }


    }

}

int main() 
{
    // cv::Mat类型确定屏幕大小
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
