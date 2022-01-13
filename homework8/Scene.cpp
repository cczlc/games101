//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    // 判断这条光线是否会打到场景中的物体
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        // 计算光源的总面积
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }

    // 这里是确定采样哪个发光物体
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;

    // 找到采样点p对应的光源模型
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }
    return (*hitObject != nullptr);
}

//Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // 判断是否打到物体
    // 这里计算了两次，还会有优化的空间！！
    Intersection inter = intersect(ray);

    if (inter.happened)
    {
        // 如果射线打到光源，直接返回
        if (inter.m->hasEmission())
        {
            // 除非depth=0，否则不可能进入这里
            return inter.m->getEmission();
        }

        Vector3f L_dir(0.0f, 0.0f, 0.0f);
        Vector3f L_indir(0.0f, 0.0f, 0.0f);

        Intersection lightInter;
        float pdf_light = 0.0f;

        // 随机采样光源上的某一点，并计算该点的概率密度
        sampleLight(lightInter, pdf_light);

        Vector3f objectNormal = inter.normal;
        Vector3f lightNormal = lightInter.normal;

        // 打到物体的位置
        Vector3f objectPos = inter.coords;
        // 随机采样点光源的位置
        Vector3f lightPos = lightInter.coords;

        Vector3f distance = lightPos - objectPos;

        // 计算点光源采样点与物体采样点之间的距离
        float dist = distance.squareNorm();
        Vector3f direction = distance.normalized();

        Ray light(objectPos, direction);
        Intersection interTemp = intersect(light);

        // 如果采样到的光源判定为遮挡，则认为直接光照项为0
        if (interTemp.happened && (interTemp.coords - lightPos).norm() < 1e-2)
        {
            Vector3f fr = inter.m->eval(ray.direction, direction, objectNormal);
            L_dir = lightInter.emit * fr * dotProduct(direction, objectNormal) * dotProduct(-direction, lightNormal) / dist / pdf_light;
        }

        if (get_random_float() < RussianRoulette)
        {
            Vector3f nextDir = inter.m->sample(ray.direction, objectNormal).normalized();
            
            // 随机产生一条光线
            Ray nextRay(objectPos, nextDir);
            // 计算机交点
            Intersection nextInter = intersect(nextRay);

            // 这个概率密度是不是不对，感觉应该是要减去灯光物体的立体角才对？？？
            if (nextInter.happened && !nextInter.m->hasEmission())
            {
                float pdf = inter.m->pdf(ray.direction, nextDir, objectNormal);
                Vector3f fr = inter.m->eval(ray.direction, nextDir, objectNormal);
                L_indir = castRay(nextRay, depth + 1) * fr * dotProduct(nextDir, objectNormal) / pdf / RussianRoulette;
            }
        }

        return L_indir + L_dir;
    }

    return Vector3f(0.0f, 0.0f, 0.0f);
}

// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
//     Vector3f L_dir = {0, 0, 0};
//     Vector3f L_indir = {0, 0, 0};

//     Intersection intersection = Scene::intersect(ray);
//     if (!intersection.happened)
//     {
//         return {};
//     }
//     //打到光源
//     if (intersection.m->hasEmission())
//         return intersection.m->getEmission();

//     //打到物体后对光源均匀采样
//     Intersection lightpos;
//     float lightpdf = 0.0f;
//     sampleLight(lightpos, lightpdf); //获得对光源的采样，包括光源的位置和采样的pdf

//     Vector3f collisionlight = lightpos.coords - intersection.coords;
//     float dis = collisionlight.norm();
//     Vector3f collisionlightdir = collisionlight.normalized();
//     Ray objray(intersection.coords, collisionlightdir);

//     Intersection ishaveobj = Scene::intersect(objray);
//     //L_dir = L_i * f_r * cos_theta * cos_theta_x / |x - p | ^ 2 / pdf_light
//     if (ishaveobj.distance - collisionlight.norm() > -EPSILON) //说明之间没有遮挡
//         L_dir = lightpos.emit * intersection.m->eval(ray.direction, collisionlightdir, intersection.normal) * dotProduct(collisionlightdir, intersection.normal) * dotProduct(-collisionlightdir, lightpos.normal) / dis / lightpdf;

//     //打到物体后对半圆随机采样使用RR算法
//     if (get_random_float() > RussianRoulette)
//         return L_dir;

//     Vector3f w0 = intersection.m->sample(ray.direction, intersection.normal).normalized();
//     Ray objrayobj(intersection.coords, w0);
//     Intersection islight = Scene::intersect(objrayobj);
//     // shade(q, wi) * f_r * cos_theta / pdf_hemi / P_RR
//     if (islight.happened && !islight.m->hasEmission())
//     {
//         float pdf = intersection.m->pdf(ray.direction, w0, intersection.normal);
//         L_indir = castRay(objrayobj, depth + 1) * intersection.m->eval(ray.direction, w0, intersection.normal) * dotProduct(w0, intersection.normal) / pdf / RussianRoulette;
//     }

//     return L_dir + L_indir;
// }
