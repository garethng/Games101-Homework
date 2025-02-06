//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // shade(p, wo)
    //     sampleLight(inter , pdf_light)
    //     Get x, ws, NN, emit from inter
    //     Shoot a ray from p to x
    //     If the ray is not blocked in the middle
    //     L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
    //     NN) / |x-p|^2 / pdf_light
    //     L_indir = 0.0
    //     Test Russian Roulette with probability RussianRoulette
    //     wi = sample(wo, N)
    //     Trace a ray r(p, wi)
    //     If ray r hit a non-emitting object at q
    //     L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
    //     / pdf(wo, wi, N) / RussianRoulette
    //     Return L_dir + L_indir
    if (depth > maxDepth) {
        return Vector3f(0.0f);
    }
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return backgroundColor;
    }
    if (inter.obj->hasEmit()) {
        return inter.emit;
    }
    Vector3f L_dir = Vector3f(0.0f);
    Vector3f L_indir = Vector3f(0.0f);
    Vector3f p = ray.origin;
    Vector3f wo = ray.direction;

    Vector3f x = inter.coords;
    Vector3f N = inter.normal;
    Vector3f ws = normalize(inter.emit - x);

    float pdf = inter.m->pdf(wo, ws, N);
    float distance = (x - p).norm();
    sampleLight(inter, pdf);

    Ray shadowRay(p, ws);
    Intersection shadowInter = intersect(shadowRay);
    
    // 检查是否被遮挡（注意要留一个小的容差值epsilon，避免自遮挡）
    bool isBlocked = shadowInter.happened && 
                     shadowInter.distance + 1e-3 < distance;
    if (!isBlocked) {
        L_dir = inter.emit;
        Vector3f f_r = inter.m->eval(wo, ws, N);   
        Vector3f N_prime = inter.normal;
        float cos_theta = dotProduct(ws, N);
        float cos_theta_prime = dotProduct(ws, N_prime);
        float pdf_light = pdf;
        L_dir = L_dir * f_r * cos_theta * cos_theta_prime / pow(distance, 2) / pdf_light;
    }


    float p_RR = get_random_float();
    if (p_RR < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, N);
        Ray ray_indir = Ray(x, wi);
        Intersection inter_indir = intersect(ray_indir);
        if (inter_indir.happened && !inter_indir.obj->hasEmit()) {
            L_indir = castRay(ray_indir, depth + 1) * inter.m->eval(wo, wi, N) * dotProduct(wi, N) / inter.m->pdf(wo, wi, N) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}
