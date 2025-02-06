#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
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

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

struct BucketInfo {
           int count = 0;
           Bounds3 bounds;
       };

struct splitInfo {
    double cost;
    double splitPos;
    int splitAxis;
    std::vector<Object*> leftObjects;
    std::vector<Object*> rightObjects;
};

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node

    if (splitMethod == SplitMethod::NAIVE) {
        Bounds3 bounds;
        for (int i = 0; i < objects.size(); ++i)
            bounds = Union(bounds, objects[i]->getBounds());
        if (objects.size() == 1) {
            // Create leaf _BVHBuildNode_
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else if (objects.size() == 2) {
            node->left = recursiveBuild(std::vector{objects[0]});
            node->right = recursiveBuild(std::vector{objects[1]});

            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
        else {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }
    else if (splitMethod == SplitMethod::SAH) {
        Bounds3 bounds;
        printf("SAH\n");
        for (int i = 0; i < objects.size(); ++i)
            bounds = Union(bounds, objects[i]->getBounds());
        if (objects.size() == 1) {
            // Create leaf _BVHBuildNode_
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else if (objects.size() == 2) {
            node->left = recursiveBuild(std::vector{objects[0]});
            node->right = recursiveBuild(std::vector{objects[1]});

            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
        else {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
            
            splitInfo minSplitInfo = {std::numeric_limits<double>::infinity(), 0, 0};
            constexpr size_t nBuckets = 30;
            for (int splitAxis = 0; splitAxis < 3; splitAxis++) {
                double width = centroidBounds.Diagonal()[splitAxis];
               std::array<BucketInfo, nBuckets> buckets;
               switch (splitAxis) {
                    case 0:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().x <
                                f2->getBounds().Centroid().x;
                        });
                        break;
                    case 1:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().y <
                                f2->getBounds().Centroid().y;
                        });
                        break;
                    case 2:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().z <
                                f2->getBounds().Centroid().z;
                        });
                        break;
                    }
                int last_j = 0;
                for (int i = 0; i < nBuckets; i++) {
                    double start = centroidBounds.pMin[splitAxis] + i * width / nBuckets;
                    double end = centroidBounds.pMin[splitAxis] + (i + 1) * width / nBuckets;
                    if (end > centroidBounds.pMax[splitAxis]) {
                        end = centroidBounds.pMax[splitAxis];
                    }
                    for (int j = last_j; j < objects.size(); j++) {
                        if (objects[j]->getBounds().Centroid()[splitAxis] >= start && objects[j]->getBounds().Centroid()[splitAxis] < end) {
                            buckets[i].count++;
                            buckets[i].bounds = Union(buckets[i].bounds, objects[j]->getBounds());
                        }else{
                            last_j = j;
                            break;
                        }
                    }
                }
                std::array<double, nBuckets - 1> cost;
                for (int i = 0; i < nBuckets - 1; i++) {
                    Bounds3 b0, b1;
                    for (int j = 0; j <= i; j++) {
                        b0 = Union(b0, buckets[j].bounds);

                    }
                    for (int j = i + 1; j < nBuckets; j++) {
                        b1 = Union(b1, buckets[j].bounds);
                    }
                    cost[i] = (b0.SurfaceArea() * buckets[i].count + b1.SurfaceArea() * (buckets[i + 1].count - buckets[i].count)) / bounds.SurfaceArea();
                }
                int minCostIndex = std::min_element(cost.begin(), cost.end()) - cost.begin();
                double splitCost = cost[minCostIndex];
                if (splitCost < minSplitInfo.cost) {
                    std::vector<Object*> leftObjects, rightObjects;
                    leftObjects = std::vector<Object*>(objects.begin(), objects.begin() + minCostIndex + 1);
                    rightObjects = std::vector<Object*>(objects.begin() + minCostIndex + 1, objects.end());
                    minSplitInfo = {splitCost, (minCostIndex + 0.5f) * width / nBuckets, splitAxis, leftObjects, rightObjects};
                }
            }

            node->left = recursiveBuild(minSplitInfo.leftObjects);
            node->right = recursiveBuild(minSplitInfo.rightObjects);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }
    

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;
    std::vector<BVHBuildNode*> nodes_queue;
    nodes_queue.push_back(node);
    float closest_so_far = ray.t_max;  // 记录当前找到的最近交点距离
//最关键的问题是遍历顺序不正确。当前实现使用队列方式遍历，但没有考虑相交点的距离顺序，这可能导致错过更近的相交点。
// 当找到一个相交点时立即返回，但这个相交点可能不是最近的。
    while (!nodes_queue.empty()) {
        BVHBuildNode* curr_node = nodes_queue.back();
        nodes_queue.pop_back();
        
        std::array<int, 3> dirIsNeg = {ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0};
        
        // 如果与包围盒不相交，或者包围盒的最近点比当前最近交点还远，则跳过
        if (!curr_node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
            continue;
        }

        if (curr_node->left == nullptr && curr_node->right == nullptr) {
            // 叶子节点，计算与物体的交点
            Intersection temp = curr_node->object->getIntersection(ray);
            // 更新最近交点
            if (temp.happened && temp.distance < closest_so_far) {
                closest_so_far = temp.distance;
                isect = temp;
            }
        } else {
            // 非叶子节点，将子节点加入队列
            if (curr_node->left) nodes_queue.push_back(curr_node->left);
            if (curr_node->right) nodes_queue.push_back(curr_node->right);
        }
    }
    return isect;
}
