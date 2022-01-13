#include <transform.h>

transform::transform(Eigen::Vector3f &point, const Eigen::Matrix3f &offset, const float angle) : 
    mPoint(point), mOffset(offset), mAngle(angle)
{
    // 根据角度确定旋转矩阵
    mRotate << cos(mAngle / 180.0 * acos(-1)), -sin(mAngle / 180.0 * acos(-1)), 0,
        sin(mAngle / 180.0 * acos(-1)), cos(mAngle / 180.0 * acos(-1)), 0,
        0, 0, 1;
}

Eigen::Vector3f transform::doTransForm()
{
    return mOffset * mRotate * mPoint;
}