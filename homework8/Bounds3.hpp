//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box

    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);        // 初始化时是形成一个最小的包围盒
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }

    // 计算包围盒最大的维度
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const
{
   // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

    // 这里没有考虑正负值的问题，一旦光线不是沿着+x，+y，+z方向进来就会出问题
    // 光线与pmax对应的三个面的交点时间（这样就不用考虑光线的方向）
    // Vector3f t1 = (pMax - ray.origin) * invDir;
    // // 光线与pmin对应的三个面的交点时间
    // Vector3f t2 = (pMin - ray.origin) * invDir;

    // // 两个相对的面要取一个较小的值（进入）
    // Vector3f tEnter = Vector3f::Min(t1, t2);
    // // 两个相对的面要取一个较大的值（离开）
    // Vector3f tExit = Vector3f::Max(t1, t2);

    // // 找到公共时间
    // float tmin = std::max(tEnter.x, std::max(tEnter.y, tEnter.z));
    // float tmax = std::min(tExit.x, std::min(tExit.y, tExit.z));

    // if(tmin <= tmax && tmax > 0)
    // {
    //     return true;
    // }

    // return false;

    // 尝试了使用向量计算会慢一倍！
    float t_Min_x = (pMin.x - ray.origin.x) * invDir[0];
    float t_Min_y = (pMin.y - ray.origin.y) * invDir[1];
    float t_Min_z = (pMin.z - ray.origin.z) * invDir[2];
    float t_Max_x = (pMax.x - ray.origin.x) * invDir[0];
    float t_Max_y = (pMax.y - ray.origin.y) * invDir[1];
    float t_Max_z = (pMax.z - ray.origin.z) * invDir[2];

    if (!dirIsNeg[0])      // 光线的x为负
    {
        float t = t_Min_x;
        t_Min_x = t_Max_x;
        t_Max_x = t;
    }
    if (!dirIsNeg[1])      // 光线的y为负
    {
        float t = t_Min_y;
        t_Min_y = t_Max_y;
        t_Max_y = t;
    }
    if (!dirIsNeg[2])      // 光线的z为负
    {
        float t = t_Min_z;
        t_Min_z = t_Max_z;
        t_Max_z = t;
    }

    float t_enter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    float t_exit = std::min(t_Max_x, std::min(t_Max_y, t_Max_z));
    
    if (t_enter <= t_exit && t_exit > 0)
        return true;
    else
        return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    // 分别取x，y，z中的较小值
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    // 分别取x，y，z中的较大值
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
