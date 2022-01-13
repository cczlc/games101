#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    root = recursiveBuild(primitives);
    time(&stop);

    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    // 计算整个场景的一个大的包围盒（或者是整个模型的大包围盒）
    // 这里好像会存在重复计算！！因为模型的大包围盒我们已经有了
    // 而且这里计算得到的大包围盒好像没有用到！
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        // 生成子节点
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        // 该模型的表面积，也是组成模型的三角形的表面积
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);

        node->area = node->left->area + node->right->area;
        return node;
    }
    else
    {   
        // 用这个来计算维度应该会更加准确
        // 这是一个以物体中点为顶点，包含所有物体中点的包围盒？？？
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        
        // 包围盒中得到最大的维度
        int dim = centroidBounds.maxExtent();

        // 根据该维度对物体进行排序
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z; });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        // 实现将一个大包围盒里的物体划分成两份
        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        // 这是一步检错
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        // 计算该节点占有的面积
        node->area = node->left->area + node->right->area;
    }

    // 是不是可以放到条件判断语句中
    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // TODO Traverse the BVH to find intersection

    Intersection intersect;

    if (!node)
    {
        return intersect;
    }

    Vector3f invdir(1. / ray.direction.x, 1. / ray.direction.y, 1. / ray.direction.z);

    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = ray.direction.x > 0;
    dirIsNeg[1] = ray.direction.y > 0;
    dirIsNeg[2] = ray.direction.z > 0;

    if (!node->bounds.IntersectP(ray, invdir, dirIsNeg))
    {
        // 如果包围盒与材料没有交点，则返回一个空交点
        return intersect;
    }

    if (!node->left && !node->right)
    {
        return node->object->getIntersection(ray);
    }

    Intersection h1 = getIntersection(node->left, ray);
    Intersection h2 = getIntersection(node->right, ray);

    return h1.distance < h2.distance ? h1 : h2;
}

void BVHAccel::getSample(BVHBuildNode *node, float p, Intersection &pos, float &pdf)
{
    // 如果BVH的加速结构的左右两个子节点有一个为空
    // 由于我们在创建BVH加速结构的时候是一个平衡二叉树，才可以这么判断
    // 这里不可能出现只有一个子节点的情况！要么是有两个子节点，要么是一个子节点都没有（待验证）
    if (node->left == nullptr || node->right == nullptr)
    {   
        // 这里的叶节点中存放了一个三角形
        node->object->Sample(pos, pdf);

        // 为什么先除后乘？？？
        pdf *= node->area;
        return;
    }

    // 如果BVH的加速结构的左右两个子节点均存在
    if (p < node->left->area)
        getSample(node->left, p, pos, pdf);
    else
        getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf)
{
    // root->area表示三角形的面积
    // 这里为什么要开根号
    // 利用模型里定义的包围盒确定采样哪一个三角形
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);

    // 为什么只除以一个模型的面积，不应该是除所有模型的面积吗
    // 如果有多个光源不会出错吗？？？
    pdf /= root->area;
}