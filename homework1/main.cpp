#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <transform.h>

int main()
{

    // // Basic Example of cpp
    // std::cout << "Example of cpp \n";
    // float a = 1.0, b = 2.0;
    // std::cout << a << std::endl;
    // std::cout << a / b << std::endl;
    // std::cout << std::sqrt(b) << std::endl;
    // std::cout << std::acos(-1) << std::endl;
    // std::cout << std::sin(30.0 / 180.0 * acos(-1)) << std::endl;

    // // Example of vector
    // std::cout << "Example of vector \n";

    // // vector definition
    // // 向量定义
    // Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    // Eigen::Vector3f w(1.0f, 0.0f, 0.0f);

    // // vector output
    // // 向量输出
    // std::cout << "Example of output \n";
    // std::cout << v << std::endl;

    // // vector add
    // // 向量相加
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;

    // // vector scalar multiply
    // // 向量相乘
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;

    // // Example of matrix
    // std::cout << "Example of matrix \n";
    // // matrix definition
    // // 矩阵定义
    // // 注意3f代表其为3x3的矩阵
    // Eigen::Matrix3f i, j;
    // i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    // j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;

    // // matrix output
    // // 矩阵输出
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;

    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    // 定义点（2，1）
    Eigen::Vector3f point(2.0f, 1.0f, 1.0f);
    std::cout << "初始点：\n" << point << std::endl;
    // 旋转角度
    float angle = 45;
    Eigen::Matrix3f offset;
    // 平移矩阵
    offset << 1.0f, 0.0f, 1,
        0.0f, 1.0f, 2.0f,
        0.0f, 0.0f, 1.0f;
    transform tf(point, offset, angle);
    point = tf.doTransForm();
    std::cout << "变换后的点：\n" << point << std::endl;

    return 0;
}