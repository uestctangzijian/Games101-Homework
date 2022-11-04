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

// 对场景中的所有光源进行随机采样
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
    Vector3f l_dir = { 0.0, 0.0, 0.0 }; // 直接光照
    Vector3f l_indir = { 0.0, 0.0, 0.0 }; //间接光照

    // 1. 判断光线是否与场景中的物体相交
    Intersection inter = Scene::intersect(ray);
    // 如果没交点
    if (!inter.happened)
    {
        return l_dir;
    }
    // 2. 如果与光源相交，渲染方程只用算自发光项
    if (inter.m->hasEmission())
    {
        if (depth == 0) // 第一个打到光源
        {
            return inter.m->getEmission();
        }
        else // 弹射打到光源
        {
            return l_dir;
        }
    }
    // 3. 如果光线打到物体，执行伪代码后面的步骤
    Intersection light_pos;
    float pdf_light = 0.0f;
    sampleLight(light_pos, pdf_light);

    // 3.1 计算直接光照

    // 物体参数
    Vector3f p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction;
    // 光源参数
    Vector3f x = light_pos.coords;
    Vector3f NN = light_pos.normal.normalized();
    Vector3f ws = (x - p).normalized();
    float dis = (x - p).norm();
    float dis2 = dotProduct((x - p), (x - p));

    // 从光源发出一条射线到物体
    Ray light_to_object(p, ws);
    Intersection light_to_scene = Scene::intersect(light_to_object);
    if (light_to_scene.happened && (light_to_scene.distance - dis > -0.005)) {
        Vector3f L_i = light_pos.emit; // 光强
        Vector3f f_r = inter.m->eval(wo, ws, N); // 材质，BDRF = 材质
        float cos_theta = dotProduct(ws, N); // 光线与物体法向量的夹角
        float cos_theta_l = dotProduct(-ws, NN); // 光线与光源表面法向量的夹角
        l_dir = L_i * f_r * cos_theta * cos_theta_l / dis2 / pdf_light;
    }

    // 3.2 间接光照
    // 俄罗斯轮盘赌
    float ksi = get_random_float();
    if (ksi < RussianRoulette) {
        // 随机生成wi方向
        Vector3f wi = inter.m->sample(wo, N).normalized();
        Ray r(p, wi);
        Intersection obj_to_scene = Scene::intersect(r);
        // 如果击中了物体且不是光源
        if (obj_to_scene.happened && !obj_to_scene.m->hasEmission()) {
            Vector3f f_r = inter.m->eval(wo, wi, N);
            float cos_theta = dotProduct(wi, N);
            float pdf_hemi = inter.m->pdf(wo, wi, N);
            l_indir = castRay(r, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
    return l_dir + l_indir;
}