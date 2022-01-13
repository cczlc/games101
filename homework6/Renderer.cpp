#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float &deg)
{
    return deg * M_PI / 180.0;
}

// Compute reflection direction
// 计算反射方向！
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    // 判断光线是在物体内还是在物体外
    float cosi = clamp(-1, 1, dotProduct(I, N)); // 保证点乘的结果在【-1, 1】之间
    float etai = 1, etat = ior;                  // etat始终表示物体
    Vector3f n = N;                              // 法向量
    if (cosi < 0)
    {
        cosi = -cosi;
    }
    else // 光线在物体里面穿出
    {
        std::swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);

    // 计算折射方向（这个公式是怎么推导出来的？）
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

// [comment]
// Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
// 利用入射光方向、法线、折射率即可计算出反射率
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;

    if (cosi > 0)
    {
        std::swap(etai, etat);
    }

    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));

    // Total internal reflection
    // 不发生折射！
    if (sint >= 1)
    {
        return 1;
    }
    else
    {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.  离投影平面最近的物体的距离
// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.   相交三角形的索引
// \param[out] uv stores the u and v barycentric coordinates of the intersected point   相交点的中心坐标？
// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.) 相交物体的指针
// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// [/comment]

std::optional<hit_payload> trace(
    const Vector3f &orig, const Vector3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects)
{
    float tNear = kInfinity;
    std::optional<hit_payload> payload;

    for (const auto &object : objects)
    {
        float tNearK = kInfinity; // 初始化为最大值
        uint32_t indexK;
        Vector2f uvK;

        // 找到相交物体的交点已经该物体的一系列信息
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            // 只有与物体有交点的时候才会分配空间！
            payload.emplace();
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK; // 球时这个值为空
            payload->uv = uvK;       // 球时这个值为空
            tNear = tNearK;
        }
    }

    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
// 如果材质是反射或者折射材质，那么我们计算反射或者折射方向，并发射光线
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]

Vector3f castRay(
    const Vector3f &orig, const Vector3f &dir, const Scene &scene,
    int depth)
{
    // 最多5次递归（即光线最多弹射5次）
    if (depth > scene.maxDepth)
    {
        return Vector3f(0.0, 0.0, 0.0);
    }

    // 将颜色初始化为屏幕的背景颜色
    Vector3f hitColor = scene.backgroundColor;

    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear;
        Vector3f N;                                                                                // normal
        Vector2f st;                                                                               // st coordinates 重心坐标，初始值为0，0
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st); // 得到该点对应的一些属性

        // 根据打到点的材质类型分别进行处理！
        switch (payload->hit_obj->materialType)
        {
        case REFLECTION_AND_REFRACTION: // 反射和折射都有的情况！
        {
            Vector3f reflectionDirection = normalize(reflect(dir, N));                        // 计算反射方向
            Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior)); // 计算折射方向，需要用到折射率！

            // 做一个小的偏移，防止算出的交点在原来的平面上
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint - N * scene.epsilon : hitPoint + N * scene.epsilon;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * scene.epsilon : hitPoint + N * scene.epsilon;

            // 递归计算颜色！
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);

            float kr = fresnel(dir, N, payload->hit_obj->ior);

            // 对于反射和折射都存在的情况，不需要加上物体本身的颜色吗？
            // 不需要，因为既然存在折射，那么他就是偏透明的
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);

            break;
        }
        case REFLECTION: // 只有反射的情况
        {
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            Vector3f reflectionDirection = reflect(dir, N);
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;

            // 为什么不用计算物体表面的颜色？？
            hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
            break;
        }
        default:
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]
            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (auto &light : scene.get_lights())
            {
                Vector3f lightDir = light->position - hitPoint;
                // square of the distance between hitPoint and the light
                float lightDistance2 = dotProduct(lightDir, lightDir);
                lightDir = normalize(lightDir);

                // 不大于0的话说明在背面！
                float LdotN = std::max(0.f, dotProduct(lightDir, N));
                // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                // 需要判断打的物体不是在光源的另一边
                bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);
                
                // 由于是raytracing，因此不需要考虑环境光照那一项
                lightAmt += inShadow ? 0 : light->intensity * LdotN;
                Vector3f reflectionDirection = reflect(-lightDir, N);

                specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                                      payload->hit_obj->specularExponent) *
                                 light->intensity;
            }

            hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
            break;
        }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));          // 视角
    float imageAspectRatio = scene.width / (float)scene.height; // 屏幕的宽高比

    // Use this variable as the eye position to start your rays.
    // 将相机位置放置在原点位置
    Vector3f eye_pos(0);
    int m = 0;

    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*

            // generate primary ray direction
            // 这里是因为z是-1所以才可以直接 x 半角
            float x = (((i + 0.5) * 2 / ((float)scene.width)) - 1.0f) * imageAspectRatio * scale;
            // 为什么要反向？ 目的是为了适应openCV（因为我们给的屏幕上的点都是openCV屏幕上的坐标）
            float y = (1.0f - 2 * (j + 0.5) / (float)scene.height) * scale;
            //float y = (2 * (j + 0.5) / (float)scene.height - 1.0f) * scale;

            // 为什么z为-1
            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            dir = normalize(dir);              // 注意要归一化
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }

        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
