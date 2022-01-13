//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    // 创建BVH结构
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
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

// Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // 注意弹射次数过多时，返回的不是背景颜色，而是【0 0 0】，相当于不计算
    if (depth > this->maxDepth)
    {
        return Vector3f(0.0, 0.0, 0.0);
    }
    // 交点相关属性
    Intersection intersection = Scene::intersect(ray);
    // 交点材质
    Material *m = intersection.m;
    // 交点对应物体（实际上是打到的三角形）
    Object *hitObject = intersection.obj;


    // 如果没打到模型，则将颜色设置为背景颜色
    Vector3f hitColor = this->backgroundColor;
    // float tnear = kInfinity;

    Vector2f uv;
    uint32_t index = 0;
    if (intersection.happened)
    {
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st;                      // st coordinates

        // 对于打到的是球体，只需要取出其法向量，不需要取出其纹理坐标
        // 这句话其实可以不要，因为并没有应用纹理贴图在模型上
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
        //        Vector3f tmp = hitPoint;
        
        switch (m->getType())
        {
        case REFLECTION_AND_REFRACTION:
        {
            // 反射和折射都存在的情况！
            Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
            Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));

            // 计算反射和折射光线的起点
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;

            Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
            Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);

            float kr;
            fresnel(ray.direction, N, m->ior, kr);

            // 这一点的颜色 = 反射光线 + 折射光线
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);

            break;
        }
        case REFLECTION:
        {
            float kr;
            fresnel(ray.direction, N, m->ior, kr);
            Vector3f reflectionDirection = reflect(ray.direction, N);
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;

            // 只有反射没有折射！（为什么不加上物体本身的颜色？，而且能量不会不守恒吗，少了折射的那一部分？？？）
            hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
            break;
        }
        default:
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]

            // 这里不考虑物体的折射
            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ? hitPoint + N * EPSILON : hitPoint - N * EPSILON;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (uint32_t i = 0; i < get_lights().size(); ++i)
            {
                // 这是什么语法？
                auto area_ptr = dynamic_cast<AreaLight *>(this->get_lights()[i].get());
                if (area_ptr)
                {
                    // Do nothing for this assignment
                }
                else
                {
                    Vector3f lightDir = get_lights()[i]->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);

                    // 防止在背面
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    Object *shadowHitObject = nullptr;

                    float tNearShadow = kInfinity;

                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    // 递归遍历场景中的模型，并递归遍历模型中的三角形
                    bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                    // 似乎没有考虑点光源在两物体中间的情况？
                    lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN / lightDistance2;

                    // 计算出反射方向
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    // 该项目中实际上 specularExponent 为0
                    // 为什么都不 /r^2
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)), m->specularExponent) * get_lights()[i]->intensity;
                }
            }
            hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
            break;
        }
        }
    }

    return hitColor;
}