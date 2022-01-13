#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);

    // 最大的根结点数目创建为1
    if (primitives.empty())
        return;

    // 传进去的是一个包含所有物体的数组
    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);

    // 这里是计算划分包围盒所需要的时间，应该很短，因为只有一个模型，只需要一个包围盒
    // 既然只有一个包围盒，为什么不直接将物体的包围盒放进BVH中就好了呢？
    // 因为在物体数目很多的情况下，该算法就有些类似于二分查找，可以有O（log（Objects））的时间复杂度，而直接放的话就是O（Objects）的复杂度
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

    // 得到所有模型的最大包围盒
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    // 递归调用，实现包围盒细分
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        // 只有两个模型时不需要判断哪个维度更大，只需要将这两个模型划分为两个包围盒
        // 创建新数组
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;

        // 遍历所有的模型（在该场景中只有一直兔子，要对该模型进行细分，如果有其他模型也可以切分其他模型）
        // 得到包围所有模型的那个总的包围盒
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        int dim = centroidBounds.maxExtent(); // 得到最大的维度

        switch (dim)
        {
            // 对该纬度的模型进行排序
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

        // 在本场景中只有一个模型
        auto beginning = objects.begin(); // 得到指向起始模型的指针
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        // 利用vector的迭代器初始化
        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        // 只有一个模型时，好像不能进行切分了，但是如果都不与包围盒有交点，那更不能与模型有交点，不需要每条光线都判断与三角形是否相交
        // 对于与包围盒有交点的光线，还是需要每一条都进行判断
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        // 对两个包围盒进行合并，但是为什么不直接用前面得到的 centroidBounds，这两个方式有什么区别吗？
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    // 没有包围盒的情况
    if (!root)
        return isect; // 存放交点位置的各种属性

    // 从这里开始，传入BVH的根节点
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // Intersection inter;

    // if(!node)
    // {
    //     return inter;
    // }
    // // TODO Traverse the BVH to find intersection
    // // 递归判断包围盒是否与光线有交点
    // Vector3f invDir(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);

    // if(node->bounds.IntersectP(ray, invDir))
    // {
    //     if(!node->left && !node->right)
    //     {
    //         // 在本个项目中调用到由三角形构成的模型的加速结构
    //         // 这里是一个object只对应一个三角形才可以这么写，不然的会需要对小节点里的所有模型进行遍历找到最小的！
    //         return node->object->getIntersection(ray);
    //     }
    //     else
    //     {
    //         Intersection inter1 = BVHAccel::getIntersection(node->left, ray);
    //         Intersection inter2 = BVHAccel::getIntersection(node->left, ray);

    //         // 存在一条光线穿过多个物体（注意这里一个包围盒只会有一个三角形）
    //         // 这里会进行递归的判断，找到整个模型里交点离屏幕最近的那个三角形的属性
    //         return inter1.distance < inter2.distance ? inter1 : inter2;
    //     }
    // }
    // else
    // {
    //     return inter;
    // }

    // Vector3f invDir(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);
    // std::array<int, 3> dirIsNeg;
    // dirIsNeg[0] = ray.direction.x>0;
    // dirIsNeg[1] = ray.direction.y>0;
    // dirIsNeg[2] = ray.direction.z>0;
    // if (node->bounds.IntersectP(ray, invDir, dirIsNeg))
    // {
    //     if (node->left == nullptr && node->right == nullptr)
    //     {
    //         // 在本个项目中调用到由三角形构成的模型的加速结构
    //         // 这里是一个object只对应一个三角形才可以这么写，不然的会需要对小节点里的所有模型进行遍历找到最小的！
    //         return node->object->getIntersection(ray);
    //     }
    //     else
    //     {
    //         Intersection inter1 = BVHAccel::getIntersection(node->left, ray);
    //         Intersection inter2 = BVHAccel::getIntersection(node->left, ray);

    //         // 存在一条光线穿过多个物体（注意这里一个包围盒只会有一个三角形）
    //         // 这里会进行递归的判断，找到整个模型里交点离屏幕最近的那个三角形的属性
    //         return inter1.distance < inter2.distance ? inter1 : inter2;
    //     }
    // }
    // else
    // {
    //     return {};
    // }

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