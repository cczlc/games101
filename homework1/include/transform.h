#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

class transform
{
public:
    transform(Eigen::Vector3f &point, const Eigen::Matrix3f &offset, const float angle);
    Eigen::Vector3f doTransForm();

private:
    Eigen::Vector3f mPoint;
    Eigen::Matrix3f mOffset; // 位移
    Eigen::Matrix3f mRotate;  // 旋转矩阵
    float mAngle;            // 旋转角度
};