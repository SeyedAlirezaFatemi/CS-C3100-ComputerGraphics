#include "cameraPath.h"
#include "extra.h"
#include "quat.hpp"
#include <array>
#include <iostream>
#include <math.h>

using namespace FW;


Mat4f FW::cameraPath::GetOrientation(float t) {
    if (orientationMode) {
        int count = orientationPoints.size();
        int start = int(t * count);
        float lambda = t * count - start;

        std::array<Vec4f, 4> controlPoints = orientationPoints[start % orientationPoints.size()];
        // YOUR CODE HERE (extra)
        // Use the De Casteljau construction with spherical interpolation (slerp) to interpolate between the orientation control point
        // quaternions in the points array, and convert the interpolated quaternion to an orientation matrix.

        // With help from:
        // https://splines.readthedocs.io/en/latest/rotation/slerp.html
        // https://splines.readthedocs.io/en/latest/rotation/de-casteljau.html
        // https://www.lix.polytechnique.fr/~nielsen/WEBvisualcomputing/programs/slerp.cpp

        Quaternion q0{controlPoints[0]};
        Quaternion q1{controlPoints[1]};
        Quaternion q2{controlPoints[2]};
        Quaternion q3{controlPoints[3]};
        Quaternion slerp_0_1;
        Slerp(q0, q1, slerp_0_1, lambda);
        Quaternion slerp_1_2;
        Slerp(q1, q2, slerp_1_2, lambda);
        Quaternion slerp_2_3;
        Slerp(q2, q3, slerp_2_3, lambda);
        Quaternion pre_final_1;
        Slerp(slerp_0_1, slerp_1_2, pre_final_1, lambda);
        Quaternion pre_final_2;
        Slerp(slerp_1_2, slerp_2_3, pre_final_2, lambda);
        Quaternion final;
        Slerp(pre_final_1, pre_final_2, final, lambda);
        float matrix[16];
        final.ExportToMatrix(matrix);
        return -1.0f * makeMat4f(matrix[0], matrix[1], matrix[2], matrix[3],
                                 matrix[4], matrix[5], matrix[6], matrix[7],
                                 matrix[8], matrix[9], matrix[10], matrix[11],
                                 matrix[12], matrix[13], matrix[14], -1.0f * matrix[15]);
        // auto slerp_0_1 = slerp(controlPoints[0], controlPoints[1], t);
        // auto slerp_1_2 = slerp(controlPoints[1], controlPoints[2], t);
        // auto slerp_2_3 = slerp(controlPoints[2], controlPoints[3], t);
        // auto q = slerp(slerp(slerp_0_1, slerp_1_2, t), slerp(slerp_1_2, slerp_2_3, t), t);
        // Mat4f res;
        // res.setRow(0, Vec4f(q[0] * q[0] + q[1] * q[1] - 0.5f, q[1] * q[2] - q[0] * q[3], q[1] * q[3] + q[0] * q[2], 0.0f));
        // res.setRow(1, Vec4f(q[1] * q[2] + q[0] * q[3], q[0] * q[0] + q[2] * q[2] - 0.5f, q[2] * q[3] - q[0] * q[1], 0.0f));
        // res.setRow(2, Vec4f(q[1] * q[3] - q[0] * q[2], q[2] * q[3] + q[0] * q[1], q[0] * q[0] + q[3] * q[3] - 0.5f, 0.0f));
        // res.setRow(3, Vec4f(0.0f, 0.0f, 0.0f, 0.5f));
        // return 2.0f * res;
    } else {
        Mat4f orientation;
        int i = int(t * positionPath.size());

        orientation.setCol(0, -Vec4f(positionPath[i].B, 0));
        orientation.setCol(1, -Vec4f(positionPath[i].N, 0));
        orientation.setCol(2, -Vec4f(positionPath[i].T, .0f));
        return orientation.transposed();
    }
}

Mat4f FW::cameraPath::GetTranslation(float t) {
    int i = int(t * positionPath.size());

    Vec3f pos = positionPath[i].V;
    return Mat4f::translate(-pos);
}

Mat4f FW::cameraPath::GetWorldToCam(float t) {
    return GetOrientation(t) * GetTranslation(t);
}

void FW::cameraPath::Draw(float t, GLContext *gl, Mat4f projection) {
    mesh->draw(gl, GetWorldToCam(t), projection);
    glUseProgram(0);
}
