#include "base/Math.hpp"

#include <cmath>

// From: https://www.lix.polytechnique.fr/~nielsen/WEBvisualcomputing/programs/slerp.cpp

class Point3D {
public:
    double x, y, z;
};

class Quaternion {
public:
    double w;
    Point3D u;

    Quaternion() = default; // default constructor
    Quaternion(FW::Vec4f q) {
        w = q[0];
        u.x = q[1];
        u.y = q[2];
        u.z = q[3];
    }

    inline void Multiply(const Quaternion q) {
        Quaternion tmp;
        tmp.u.x = ((w * q.u.x) + (u.x * q.w) + (u.y * q.u.z) - (u.z * q.u.y));
        tmp.u.y = ((w * q.u.y) - (u.x * q.u.z) + (u.y * q.w) + (u.z * q.u.x));
        tmp.u.z = ((w * q.u.z) + (u.x * q.u.y) - (u.y * q.u.x) + (u.z * q.w));
        tmp.w = ((w * q.w) - (u.x * q.u.x) - (u.y * q.u.y) - (u.z * q.u.z));
        *this = tmp;
    }

    inline double Norm() { return sqrt(u.x * u.x + u.y * u.y + u.z * u.z + w * w); }


    inline void Normalize() {
        double norm = Norm();
        u.x /= norm;
        u.y /= norm;
        u.z /= norm;
    }

    inline void Conjugate() {
        u.x = -u.x;
        u.y = -u.y;
        u.z = -u.z;
    }

    inline void Inverse() {
        double norm = Norm();
        Conjugate();
        u.x /= norm;
        u.y /= norm;
        u.z /= norm;
        w /= norm;
    }

    void ExportToMatrix(float matrix[16]) {
        float wx, wy, wz, xx, yy, yz, xy, xz, zz;
        // adapted from Shoemake
        xx = u.x * u.x;
        xy = u.x * u.y;
        xz = u.x * u.z;
        yy = u.y * u.y;
        zz = u.z * u.z;
        yz = u.y * u.z;

        wx = w * u.x;
        wy = w * u.y;
        wz = w * u.z;

        matrix[0] = 1.0f - 2.0f * (yy + zz);
        matrix[4] = 2.0f * (xy - wz);
        matrix[8] = 2.0f * (xz + wy);
        matrix[12] = 0.0;

        matrix[1] = 2.0f * (xy + wz);
        matrix[5] = 1.0f - 2.0f * (xx + zz);
        matrix[9] = 2.0f * (yz - wx);
        matrix[13] = 0.0;

        matrix[2] = 2.0f * (xz - wy);
        matrix[6] = 2.0f * (yz + wx);
        matrix[10] = 1.0f - 2.0f * (xx + yy);
        matrix[14] = 0.0;

        matrix[3] = 0;
        matrix[7] = 0;
        matrix[11] = 0;
        matrix[15] = 1;
    }
};


void Slerp(Quaternion q1, Quaternion q2, Quaternion &qr, double lambda) {
    float dotproduct = q1.u.x * q2.u.x + q1.u.y * q2.u.y + q1.u.z * q2.u.z + q1.w * q2.w;
    float theta, st, sut, sout, coeff1, coeff2;

    // algorithm adapted from Shoemake's paper
    lambda = lambda / 2.0;

    theta = (float) acos(dotproduct);
    if (theta < 0.0) theta = -theta;

    st = (float) sin(theta);
    sut = (float) sin(lambda * theta);
    sout = (float) sin((1 - lambda) * theta);
    coeff1 = sout / st;
    coeff2 = sut / st;

    qr.u.x = coeff1 * q1.u.x + coeff2 * q2.u.x;
    qr.u.y = coeff1 * q1.u.y + coeff2 * q2.u.y;
    qr.u.z = coeff1 * q1.u.z + coeff2 * q2.u.z;
    qr.w = coeff1 * q1.w + coeff2 * q2.w;

    qr.Normalize();
}