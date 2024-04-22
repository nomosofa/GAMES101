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
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection isect = intersect(ray);

    if (!isect.happened)
        return Vector3f();

    if (isect.m->hasEmission())
    {
        return isect.m->getEmission();
    }

    Vector3f p = isect.coords;
    // Vector3f wo = ray.origin - p;
    const Vector3f wo = -ray.direction;
    Vector3f N = normalize(isect.normal);

    // Direct
    float pdf;
    Intersection pos;
    sampleLight(pos, pdf); // coords, normal, emit stored in pos
    Vector3f l_dir;
    Vector3f dir_p_to_light = pos.coords - p;
    Vector3f wi = normalize(dir_p_to_light);
    // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
    Intersection testReachedLightSource = intersect({p, wi});
    // check if blocked
    if (testReachedLightSource.distance - dir_p_to_light.norm() > -5e-4)
    {
        Vector3f fr = isect.m->eval(wi, wo, N);
        float dot1 = std::max(.0f, dotProduct(wi, N));
        float dot2 = std::max(.0f, dotProduct(-wi, pos.normal));
        l_dir = pos.emit * fr * dot1 * dot2 / dotProduct(dir_p_to_light, dir_p_to_light) / pdf;
    }

    // Indirect
    Vector3f l_indir;
    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = isect.m->sample(wo, N).normalized();
        float pdf = isect.m->pdf(wo, wi, N);
        if (pdf > 5e-4)
        {
            Intersection testReachedNonEmit = intersect({p, wi});
            if (testReachedNonEmit.happened && !testReachedNonEmit.m->hasEmission())
            {
                Vector3f shade_q = castRay({testReachedNonEmit.coords, wi}, depth + 1);
                Vector3f fr = isect.m->eval(wi, wo, N);
                float dot = std::max(.0f, dotProduct(wi, N));
                l_indir = shade_q * fr * dot / pdf / RussianRoulette;
            }
        }
    }
    return l_dir + l_indir;
}