#include "objects.hpp"

#include "VecUtils.h"
#include "hit.hpp"

#include <cassert>

using namespace std;
using namespace FW;

Object3D *Group::operator[](int i) const {
    assert(i >= 0 && size_t(i) < size());
    return objects_[i].get();
}

void Group::insert(Object3D *o) {
    assert(o);
    objects_.emplace_back(o);
}

bool Group::intersect(const Ray &r, Hit &h, float tmin) const {
    // We intersect the ray with each object contained in the group.
    bool intersected = false;
    for (int i = 0; i < int(size()); ++i) {
        Object3D *o = objects_[i].get();
        assert(o != nullptr);
        assert(h.t >= tmin);
        bool tmp = o->intersect(r, h, tmin);
        assert(h.t >= tmin);
        if (tmp)
            intersected = true;
    }
    assert(h.t >= tmin);
    return intersected;
}

bool Box::intersect(const Ray &r, Hit &h, float t_min) const {
    // YOUR CODE HERE (EXTRA)
    // Intersect the box with the ray!
    // With lots of help from https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    Vec3f invdir = 1.0f / r.direction;
    // Vec3f normal(0.0f, 1.0f, 0.0f);
    Vec3f normal(1.0f, 0.0f, 0.0f);
    if (invdir.x >= 0) {
        tmin = (min_.x - r.origin.x) * invdir.x;
        tmax = (max_.x - r.origin.x) * invdir.x;
    } else {
        tmin = (max_.x - r.origin.x) * invdir.x;
        tmax = (min_.x - r.origin.x) * invdir.x;
        normal *= -1;
    }
    if (invdir.y >= 0) {
        tymin = (min_.y - r.origin.y) * invdir.y;
        tymax = (max_.y - r.origin.y) * invdir.y;
    } else {
        tymin = (max_.y - r.origin.y) * invdir.y;
        tymax = (min_.y - r.origin.y) * invdir.y;
    }


    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin) {
        tmin = tymin;
        normal = (invdir.y >= 0) ? Vec3f(0.0f, 1.0f, 0.0f) : Vec3f(0.0f, -1.0f, 0.0f);
    }
    if (tymax < tmax)
        tmax = tymax;

    if (invdir.z >= 0) {
        tymin = (min_.z - r.origin.z) * invdir.z;
        tymax = (max_.z - r.origin.z) * invdir.z;
    } else {
        tymin = (max_.z - r.origin.z) * invdir.z;
        tymax = (min_.z - r.origin.z) * invdir.z;
    }

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin) {
        tmin = tzmin;
        normal = (invdir.z >= 0) ? Vec3f(0.0f, 0.0f, 1.0f) : Vec3f(0.0f, 0.0f, -1.0f);
    }
    if (tzmax < tmax)
        tmax = tzmax;
    if (tmin > t_min) {
        h.set(tmin, this->material_, normal);
        return true;
    }
    return false;
}

bool Plane::intersect(const Ray &r, Hit &h, float tmin) const {
    // YOUR CODE HERE (R5)
    // Intersect the ray with the plane.
    // Pay attention to respecting tmin and h.t!
    // Equation for a plane:
    // ax + by + cz = d;
    // normal . p - d = 0
    // (plug in ray)
    // origin + direction * t = p(t)
    // origin . normal + t * direction . normal = d;
    // t = (d - origin . normal) / (direction . normal);
    if (r.direction.dot(this->normal())) {
        auto t = (this->offset_ - r.origin.dot(this->normal_)) / (this->normal_.dot(r.direction));
        if (t >= tmin && t < h.t) {
            h.set(t, this->material_, this->normal_);
            return true;
        }
    }
    return false;
}

Transform::Transform(const Mat4f &m, Object3D *o) : matrix_(m),
                                                    object_(o) {
    assert(o != nullptr);
    inverse_ = matrix_.inverted();
    inverse_transpose_ = inverse_.transposed();
}

bool Transform::intersect(const Ray &r, Hit &h, float tmin) const {
    // YOUR CODE HERE (EXTRA)
    // Transform the ray to the coordinate system of the object inside,
    // intersect, then transform the normal back. If you don't renormalize
    // the ray direction, you can just keep the t value and do not need to
    // recompute it!
    // Remember how points, directions, and normals are transformed differently!
    Ray new_ray{this->inverse_ * Vec4f(r.origin, 1.0f).getXYZ(), (this->inverse_ * Vec4f(r.direction, 0.0f)).getXYZ()};
    auto result = this->object_.get()->intersect(new_ray, h, tmin);
    if (result) {
        h.set(h.t, h.material, FW::normalize((this->inverse_transpose_ * Vec4f(h.normal, 0.0f)).getXYZ()));
        return result;
    }
    return false;
}

bool Sphere::intersect(const Ray &r, Hit &h, float tmin) const {
    // Note that the sphere is not necessarily centered at the origin.

    Vec3f tmp = center_ - r.origin;
    Vec3f dir = r.direction;

    float A = dot(dir, dir);
    float B = -2 * dot(dir, tmp);
    float C = dot(tmp, tmp) - sqr(radius_);
    float radical = B * B - 4 * A * C;
    if (radical < 0)
        return false;

    radical = sqrtf(radical);
    float t_m = (-B - radical) / (2 * A);
    float t_p = (-B + radical) / (2 * A);
    Vec3f pt_m = r.pointAtParameter(t_m);
    Vec3f pt_p = r.pointAtParameter(t_p);

    assert(r.direction.length() > 0.9f);

    bool flag = t_m <= t_p;
    if (!flag) {
        ::printf("sphere ts: %f %f %f\n", tmin, t_m, t_p);
        return false;
    }
    assert(t_m <= t_p);

    // choose the closest hit in front of tmin
    float t = (t_m < tmin) ? t_p : t_m;

    if (h.t > t && t > tmin) {
        Vec3f normal = r.pointAtParameter(t);
        normal -= center_;
        normal.normalize();
        h.set(t, this->material(), normal);
        return true;
    }
    return false;
}

Triangle::Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c,
                   Material *m, const Vec2f &ta, const Vec2f &tb, const Vec2f &tc, bool load_mesh) : Object3D(m) {
    vertices_[0] = a;
    vertices_[1] = b;
    vertices_[2] = c;
    texcoords_[0] = ta;
    texcoords_[1] = tb;
    texcoords_[2] = tc;

    if (load_mesh) {
        preview_mesh.reset((FW::Mesh<FW::VertexPNT> *) FW::importMesh("preview_assets/tri.obj"));
        set_preview_materials();
    }
}

bool Triangle::intersect(const Ray &r, Hit &h, float tmin) const {
    // YOUR CODE HERE (R6)
    // Intersect the triangle with the ray!
    // Again, pay attention to respecting tmin and h.t!
    // Let's use barycentric coordinates
    const auto &a = this->vertices_[0];
    const auto &b = this->vertices_[1];
    const auto &c = this->vertices_[2];
    const auto &ray_dir = r.direction;
    const auto &ray_origin = r.origin;
    const auto normal = FW::normalize(FW::cross(b - a, c - a));
    if (normal.dot(ray_dir) == 0) {
        // Parallel
        return false;
    }
    FW::Mat3f A;
    A.setCol(0, a - b);
    A.setCol(1, a - c);
    A.setCol(2, ray_dir);
    FW::Vec3f right_hand = a - ray_origin;
    auto res = FW::invert(A) * right_hand;
    const auto &beta = res[0];
    const auto &gamma = res[1];
    const auto &t = res[2];

    if (beta >= 0 && gamma >= 0 && beta + gamma <= 1 && t < h.t && t > tmin) {
        h.set(t, this->material_, normal);
        return true;
    }
    return false;
}

const Vec3f &Triangle::vertex(int i) const {
    assert(i >= 0 && i < 3);
    return vertices_[i];
}
