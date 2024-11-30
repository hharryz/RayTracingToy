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
    if (depth > this->maxDepth) {
        return Vector3f(0);
    }

    //  Find the first intersection of the ray with the scene
    Intersection inter = intersect(ray);

    if (inter.emit.norm() > 0) {
        //  If the ray intersects an object that emits light, 
        //  return the light emitted by the object
        return inter.emit;
    }

    if (inter.happened) {
        //  If the ray intersects an object,
        //  calculate the color of the object at the intersection point
        Vector3f w0 = normalize(-ray.direction);
        Vector3f p = inter.coords;
        Vector3f N = normalize(inter.normal);

        float pdfLight = 0.0f;
        Intersection interLight;
        sampleLight(interLight, pdfLight);
        Vector3f x = interLight.coords;
        Vector3f ws = normalize(x - p);
        Vector3f NN = normalize(interLight.normal);

        //  direct light
        Vector3f L_dir = Vector3f(0);
        float kEpsilon = 0.01;
        if ((intersect(Ray(p, ws)).coords - x).norm() < kEpsilon) {
            //  if the ray is not blocked by other objects
            float x2pDist = (x - p).norm();
            L_dir = interLight.emit * inter.m->eval(w0, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (x2pDist * x2pDist * pdfLight);
        }
        
        //  indirect light
        Vector3f L_indir = Vector3f(0.0);
        float RR = get_random_float();
        if (RR < RussianRoulette) {
            Vector3f wi = inter.m->sample(w0, N);
            float pdf = inter.m->pdf(wi, w0, N);
            Ray r(p, wi);
            L_indir = castRay(r, depth + 1) * inter.m->eval(wi, w0, N) * dotProduct(wi, N) / (pdf * RussianRoulette);
        }

        return L_dir + L_indir;
    }

    return Vector3f(0);
}