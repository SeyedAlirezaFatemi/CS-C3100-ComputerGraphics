#include "raytracer.hpp"

#include "Camera.h"
#include "SceneParser.h"
#include "args.hpp"
#include "hit.hpp"
#include "lights.hpp"
#include "material.hpp"
#include "objects.hpp"
#include "ray.hpp"

#define EPSILON 0.001f

using namespace std;
using namespace FW;

namespace {

    // Compute the mirror direction for the incoming direction, given the surface normal.
    Vec3f mirrorDirection(const Vec3f &normal, const Vec3f &incoming) {
        // YOUR CODE HERE (R8)
        // Pay attention to the direction which things point towards, and that you only
        // pass in normalized vectors.
        return FW::normalize(incoming - 2.0f * (incoming.dot(normal)) * normal);
    }

    bool transmittedDirection(const Vec3f &normal, const Vec3f &incoming,
                              float index_i, float index_t, Vec3f &transmitted) {
        // YOUR CODE HERE (EXTRA)
        // Compute the transmitted direction for the incoming direction, given the surface normal
        // and indices of refraction. Pay attention to the direction which things point towards!
        // You should only pass in normalized vectors!
        // The function should return true if the computation was successful, and false
        // if the transmitted direction can't be computed due to total internal reflection.
        const auto cos_theta_i = FW::dot(normal, -incoming);
        const auto eta_r = index_t / index_i;
        auto temp = 1.0f - eta_r * eta_r * (1 - cos_theta_i * cos_theta_i);
        if (temp < 0) {
            return false;
        }
        transmitted = FW::normalize((eta_r * cos_theta_i - FW::sqrt(temp)) * normal + eta_r * incoming);
        return true;
    }

} // namespace

Vec3f RayTracer::traceRay(Ray &ray, float tmin, int bounces, float refr_index, Hit &hit, FW::Vec3f debug_color) const {
    // initialize a hit to infinitely far away
    hit = Hit(FLT_MAX);

    // Ask the root node (the single "Group" in the scene) for an intersection.
    bool intersect = false;
    if (scene_.getGroup())
        intersect = scene_.getGroup()->intersect(ray, hit, tmin);

    // Write out the ray segment if visualizing (for debugging purposes)
    if (debug_trace)
        debug_rays.push_back(RaySegment(ray.origin, ray.direction.normalized() * FW::min(100.0f, hit.t), hit.normal, debug_color));

    // if the ray missed, we return the background color.
    if (!intersect)
        return scene_.getBackgroundColor();

    if (ray.is_inside) {
        hit.normal = -hit.normal;
    }

    Material *m = hit.material;
    assert(m != nullptr);

    // get the intersection point and normal.
    Vec3f normal = hit.normal;
    const Vec3f point = ray.pointAtParameter(hit.t);

    // YOUR CODE HERE (R1)
    // Apply ambient lighting using the ambient light of the scene
    // and the diffuse color of the material.
    Vec3f color = m->diffuse_color(point) * this->scene_.getAmbientLight();

    // YOUR CODE HERE (R4 & R7)
    // For R4, loop over all the lights in the scene and add their contributions to the answer.
    // If you wish to include the shadow rays in the debug visualization, insert the segments
    // into the debug_rays list similar to what is done to the primary rays after intersection.
    // For R7, if args_.shadows is on, also shoot a shadow ray from the hit point to the light
    // to confirm it isn't blocked; if it is, ignore the contribution of the light.
    FW::Vec3f dir_to_light;
    FW::Vec3f incident_intensity;
    FW::Vec3f p;
    for (size_t i = 0; i < this->scene_.getNumLights(); i++) {
        float distance;
        this->scene_.getLight(i)->getIncidentIllumination(p, dir_to_light, incident_intensity, distance);
        if (this->args_.shadows) {
            Ray shadow_ray{point, dir_to_light};
            bool blocked = false;
            auto shadow_hit = Hit(FLT_MAX);
            if (scene_.getGroup()) {
                blocked = scene_.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON);
            }
            if (blocked) {
                debug_rays.emplace_back(shadow_ray.origin, shadow_ray.direction * shadow_hit.t, shadow_hit.normal, Vec3f(1.0));
                // In shadow. Don't shade!
                continue;
            }
        }
        color += m->shade(ray, hit, dir_to_light, incident_intensity, this->args_.shade_back);
    }

    // are there bounces left?
    if (bounces >= 1) {
        // reflection, but only if reflective coefficient > 0!
        if (m->reflective_color(point).length() > 0.0f) {
            // YOUR CODE HERE (R8)
            // Generate and trace a reflected ray to the ideal mirror direction and add
            // the contribution to the result. Remember to modulate the returned light
            // by the reflective color of the material of the hit point.
            const auto mirror_direction = mirrorDirection(hit.normal, ray.direction);
            Ray reflection_ray{point, mirror_direction};
            auto reflection_hit = Hit(FLT_MAX);
            color += m->reflective_color(point) * this->traceRay(reflection_ray, EPSILON, bounces - 1, refr_index, reflection_hit, Vec3f(1.0f, 1.0f, 0.0f));
        }

        // refraction, but only if surface is transparent!
        if (m->transparent_color(point).length() > 0.0f) {
            // YOUR CODE HERE (EXTRA)
            // Generate a refracted direction and trace the ray. For this, you need
            // the index of refraction of the object. You should consider a ray going through
            // the object "against the normal" to be entering the material, and a ray going
            // through the other direction as exiting the material to vacuum (refractive index=1).
            // (Assume rays always start in vacuum, and don't worry about multiple intersecting
            // refractive objects!) Remembering this will help you figure out which way you
            // should use the material's refractive index. Remember to modulate the result
            // with the material's refractiveColor().
            // REMEMBER you need to account for the possibility of total internal reflection as well.
            Vec3f transmitted_direction;
            bool did_transmit = transmittedDirection(normal, ray.direction, refr_index, ray.is_inside ? 1.0f : m->refraction_index(point), transmitted_direction);
            if (did_transmit) {
                Ray transmission_ray{ray.pointAtParameter(hit.t + EPSILON), transmitted_direction};
                transmission_ray.is_inside = !ray.is_inside;
                auto transmission_hit = Hit(FLT_MAX);
                color += m->transparent_color(point) * this->traceRay(transmission_ray, EPSILON, bounces - 1, ray.is_inside ? 1.0f : m->refraction_index(point), transmission_hit, Vec3f(1.0f, 1.0f, 0.0f));
            } else {
                const auto mirror_direction = mirrorDirection(hit.normal, ray.direction);
                Ray reflection_ray{point, mirror_direction};
                auto reflection_hit = Hit(FLT_MAX);
                color += m->transparent_color(point) * this->traceRay(reflection_ray, EPSILON, bounces - 1, refr_index, reflection_hit, Vec3f(1.0f, 1.0f, 0.0f));
            }
        }
    }
    return color;
}
