#include "surf.h"
#include "extra.h"

using namespace std;
using namespace FW;

namespace {
    // This is a generic function that generates a set of triangle
    // faces for a sweeping a profile curve along "something".  For
    // instance, say you want to sweep the profile curve [01234]:
    //
    //   4     9     10
    //    3     8     11
    //    2 --> 7 --> 12 ----------[in this direction]--------->
    //    1     6     13
    //   0     5     14
    //
    // Then the "diameter" is 5, and the "length" is how many times
    // the profile is repeated in the sweep direction.  This function
    // generates faces in terms of vertex indices.  It is assumed that
    // the indices go as shown in the picture (the first dia vertices
    // correspond to the first repetition of the profile curve, and so
    // on).  It will generate faces [0 5 1], [1 5 6], [1 6 2], ...
    // The boolean variable "closed" will determine whether the
    // function closes the curve (that is, connects the last profile
    // to the first profile).
    static vector<FW::Vec3i> triSweep(unsigned dia, unsigned len, bool closed) {
        vector<FW::Vec3i> ret;
        // We have dia * len in total.
        len--;
        // YOUR CODE HERE: generate zigzagging triangle indices and push them to ret.
        for (int i = 0; i < len; i++) {
            auto current_first = dia * i;
            auto next_first = dia * (i + 1);
            for (int j = 0; j < dia - 1; j++) {
                ret.emplace_back(current_first + j, next_first + j, current_first + j + 1);
                ret.emplace_back(current_first + j + 1, next_first + j, next_first + j + 1);
            }
            // ret.emplace_back(current_first + dia - 1, next_first + dia - 1, current_first);
            // ret.emplace_back(current_first, next_first + dia - 1, next_first);
        }
        if (closed) {
            auto current_first = len * dia;
            auto next_first = 0;
            for (int j = 0; j < dia - 1; j++) {
                ret.emplace_back(current_first + j, next_first + j, current_first + j + 1);
                ret.emplace_back(current_first + j + 1, next_first + j, next_first + j + 1);
            }
            ret.emplace_back(current_first + dia - 1, next_first + dia - 1, current_first);
            ret.emplace_back(current_first, next_first + dia - 1, next_first);
        }

        return ret;
    }

    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile) {
        for (unsigned i = 0; i < profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;

        return true;
    }
} // namespace

Surface makeSurfRev(const Curve &profile, unsigned steps) {
    Surface surface;

    if (!checkFlat(profile)) {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // YOUR CODE HERE: build the surface.  See surf.h for type details.
    // Generate the vertices and normals by looping the number of the steps and again for each
    // point in the profile (that's two cascaded loops), and finally get the faces with triSweep.
    // You'll need to rotate the curve at each step, similar to the cone in assignment 0 but
    // now you should be using a real rotation matrix.
    for (int i = 0; i < steps; i++) {
        float angle = (-2 * FW_PI * i) / (steps);
        auto rotation_matrix = Mat3f::rotation(Vec3f(0.0, 1.0, 0.0), angle);
        for (const auto &curve_point : profile) {
            surface.VV.push_back(rotation_matrix * curve_point.V);
            surface.VN.push_back(FW::normalize(rotation_matrix * -1.0f * curve_point.N));
        }
    }
    surface.VF = triSweep(profile.size(), steps, true);

    // cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;

    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep) {
    Surface surface;

    if (!checkFlat(profile)) {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // YOUR CODE HERE: build the surface.
    // This is again two cascaded loops. Build the local coordinate systems and transform
    // the points in a very similar way to the one with makeSurfRev.

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." << endl;

    return surface;
}

void drawSurface(const Surface &surface, bool shaded) {
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded) {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    } else {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glColor4f(0.4f, 0.4f, 0.4f, 1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i = 0; i < surface.VF.size(); i++) {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len) {
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0, 1, 1, 1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i = 0; i < surface.VV.size(); i++) {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface) {

    for (unsigned i = 0; i < surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i = 0; i < surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;

    for (unsigned i = 0; i < surface.VF.size(); i++) {
        out << "f  ";
        for (unsigned j = 0; j < 3; j++) {
            unsigned a = surface.VF[i][j] + 1;
            out << a << "/"
                << "1"
                << "/" << a << " ";
        }
        out << endl;
    }
}
