#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"

#include "base/Main.hpp"
#include "gpu/Buffer.hpp"
#include "gpu/GLContext.hpp"
#include "updatable_priority_queue.hpp"
#include "utility.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>

using namespace FW;
using namespace std;

// Anonymous namespace. This is a C++ way to define things which
// do not need to be visible outside this file.
namespace {

    enum VertexShaderAttributeLocations {
        ATTRIB_POSITION = 0,
        ATTRIB_NORMAL = 1,
        ATTRIB_COLOR = 2
    };

    const Vertex reference_plane_data[] = {
        {Vec3f(-1, -1, -1), Vec3f(0, 1, 0)},
        {Vec3f(1, -1, -1), Vec3f(0, 1, 0)},
        {Vec3f(1, -1, 1), Vec3f(0, 1, 0)},
        {Vec3f(-1, -1, -1), Vec3f(0, 1, 0)},
        {Vec3f(1, -1, 1), Vec3f(0, 1, 0)},
        {Vec3f(-1, -1, 1), Vec3f(0, 1, 0)}};

    vector<Vertex> loadExampleModel() {
        static const Vertex example_data[] = {
            {Vec3f(0.0f, 0.5f, 0), Vec3f(0.0f, 0.0f, -1.0f)},
            {Vec3f(-0.5f, -0.5f, 0), Vec3f(0.0f, 0.0f, -1.0f)},
            {Vec3f(0.5f, -0.5f, 0), Vec3f(0.0f, 0.0f, -1.0f)}};
        vector<Vertex> vertices;
        for (auto v : example_data) vertices.push_back(v);
        return vertices;
    }

    vector<Vertex> unpackIndexedData(
        const vector<Vec3f> &positions,
        const vector<Vec3f> &normals,
        const vector<array<unsigned, 6>> &faces) {
        vector<Vertex> vertices;

        // This is a 'range-for' loop which goes through all objects in the container 'faces'.
        // '&' gives us a reference to the object inside the container; if we omitted '&',
        // 'f' would be a copy of the object instead.
        // The compiler already knows the type of objects inside the container, so we can
        // just write 'auto' instead of having to spell out 'array<unsigned, 6>'.
        for (const auto &f : faces) {
            // YOUR CODE HERE (R3)
            // Unpack the indexed data into a vertex array. For every face, you have to
            // create three vertices and add them to the vector 'vertices'.
            // f[0] is the index of the position of the first vertex
            // f[1] is the index of the normal of the first vertex
            // f[2] is the index of the position of the second vertex
            // ...
            Vertex vertex;
            vertex.position = positions[f[0]];
            vertex.normal = normals[f[1]];
            vertices.emplace_back(vertex);
            vertex.position = positions[f[2]];
            vertex.normal = normals[f[3]];
            vertices.emplace_back(vertex);
            vertex.position = positions[f[4]];
            vertex.normal = normals[f[5]];
            vertices.emplace_back(vertex);
        }

        return vertices;
    };

    // This is for testing your unpackIndexedData implementation.
    // You should get a tetrahedron like in example.exe.
    vector<Vertex> loadIndexedDataModel() {
        static const Vec3f point_data[] = {
            Vec3f(0.0f, 0.407f, 0.0f),
            Vec3f(0.0f, -0.3f, -0.5f),
            Vec3f(0.433f, -0.3f, 0.25f),
            Vec3f(-0.433f, -0.3f, 0.25f),
        };
        static const Vec3f normal_data[] = {
            Vec3f(0.8165f, 0.3334f, -0.4714f),
            Vec3f(0.0f, 0.3334f, 0.9428f),
            Vec3f(-0.8165f, 0.3334f, -0.4714f),
            Vec3f(0.0f, -1.0f, 0.0f)};
        static const unsigned face_data[][6] = {
            {0, 0, 1, 0, 2, 0},
            {0, 2, 3, 2, 1, 2},
            {0, 1, 2, 1, 3, 1},
            {1, 3, 3, 3, 2, 3}};
        vector<Vec3f> points(point_data, point_data + SIZEOF_ARRAY(point_data));
        vector<Vec3f> normals(normal_data, normal_data + SIZEOF_ARRAY(normal_data));
        vector<array<unsigned, 6>> faces;
        for (auto arr : face_data) {
            array<unsigned, 6> f;
            copy(arr, arr + 6, f.begin());
            faces.push_back(f);
        }
        return unpackIndexedData(points, normals, faces);
    }

    // Generate an upright cone with tip at (0, 0, 0), a radius of 0.25 and a height of 1.0.
    // You can leave the base of the cone open, like it is in example.exe.
    vector<Vertex> loadUserGeneratedModel() {
        static const float radius = 0.25f;
        static const float height = 1.0f;
        static const unsigned faces = 40;
        static const float angle_increment = 2 * FW_PI / faces;

        // Empty array of Vertex structs; every three vertices = one triangle
        std::vector<Vertex> vertices;

        Vertex v0, v1, v2;
        v0.position = Vec3f{0, 0, 0};
        v1.position = Vec3f{0, -height, 0};
        v2.position = Vec3f{0, -height, 0};
        float current_angle, previous_angle;

        // Generate one face at a time
        for (auto i = 0u; i < faces; ++i) {
            // YOUR CODE HERE (R2)
            // Figure out the correct positions of the three vertices of this face.
            // v0.position = ...
            // Calculate the normal of the face from the positions and use it for all vertices.
            // v0.normal = v1.normal = v2.normal = ...;
            //
            // Some hints:
            // - Try just making a triangle in fixed coordinates at first.
            // - "FW::cos(angle_increment * i) * radius" gives you the X-coordinate
            //    of the ith vertex at the base of the cone. Z-coordinate is very similar.
            // - For the normal calculation, you'll want to use the cross() function for
            //   cross product, and Vec3f's .normalized() or .normalize() methods.
            current_angle = (i + 1) * angle_increment;
            previous_angle = i * angle_increment;
            v1.position.x = FW::cos(previous_angle) * radius;
            v1.position.z = FW::sin(previous_angle) * radius;
            v2.position.x = FW::cos(current_angle) * radius;
            v2.position.z = FW::sin(current_angle) * radius;
            v0.normal = v1.normal = v2.normal = normalize(cross(v0.position - v1.position, v0.position - v2.position));
            // Then we add the vertices to the array.
            // .push_back() grows the size of the vector by one, copies its argument,
            // and places the copy at the back of the vector.
            vertices.push_back(v0);
            vertices.push_back(v1);
            vertices.push_back(v2);
        }
        return vertices;
    }

} // namespace

App::App(void)
    : common_ctrl_(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
      current_model_(MODEL_EXAMPLE),
      model_changed_(true),
      shading_toggle_(false),
      shading_mode_changed_(false),
      camera_rotation_angle_(0.0f),
      object_rotation_angle_(0.0f),
      object_x_scale_(1.0f),
      camera_z_translation_(0.0f),
      camera_x_rotation_angle_(0.0f),
      prev_time_(0.0f),
      animating_(false),
      fov_(FW_PI / 2.0) {
    static_assert(is_standard_layout_v<Vertex>, "struct Vertex must be standard layout to use offsetof");
    initRendering();

    common_ctrl_.showFPS(true);
    common_ctrl_.addToggle((S32 *) &current_model_, MODEL_EXAMPLE, FW_KEY_1, "Triangle (1)", &model_changed_);
    common_ctrl_.addToggle((S32 *) &current_model_, MODEL_USER_GENERATED, FW_KEY_2, "Generated cone (2)", &model_changed_);
    common_ctrl_.addToggle((S32 *) &current_model_, MODEL_FROM_INDEXED_DATA, FW_KEY_3, "Unpacked tetrahedron (3)", &model_changed_);
    common_ctrl_.addToggle((S32 *) &current_model_, MODEL_FROM_FILE, FW_KEY_4, "Model loaded from file (4)", &model_changed_);
    common_ctrl_.addToggle((S32 *) &current_model_, SIMPLIFIED_MODEL_FROM_FILE, FW_KEY_5, "Model loaded from file and simplified (5)", &model_changed_);
    common_ctrl_.addSeparator();
    common_ctrl_.addToggle(&shading_toggle_, FW_KEY_T, "Toggle shading mode (T)", &shading_mode_changed_);

    window_.setTitle("Assignment 1");

    window_.addListener(this);
    window_.addListener(&common_ctrl_);

    window_.setSize(Vec2i(800, 800));
}

void App::streamGeometry(const std::vector<Vertex> &vertices) {
    // Load the vertex buffer to GPU.
    glBindBuffer(GL_ARRAY_BUFFER, gl_.dynamic_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    vertex_count_ = vertices.size();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool App::handleEvent(const Window::Event &ev) {
    if (this->animating_) {
        this->camera_rotation_angle_ += (this->timer_.getElapsed() - prev_time_) * 0.1 * FW_PI;
        this->prev_time_ = this->timer_.getElapsed();
    }
    if (model_changed_) {
        model_changed_ = false;

        switch (current_model_) {
            case MODEL_EXAMPLE:
                streamGeometry(loadExampleModel());
                break;
            case MODEL_FROM_INDEXED_DATA:
                streamGeometry(loadIndexedDataModel());
                break;
            case MODEL_USER_GENERATED:
                streamGeometry(loadUserGeneratedModel());
                break;
            case MODEL_FROM_FILE: {
                // EXTRA: Load PLY.
                auto filename = window_.showFileLoadDialog("Load new mesh");
                if (filename.getLength()) {
                    auto ext = filename.substring(filename.lastIndexOf(".") + 1).toLower();
                    if (ext == "ply") {
                        streamGeometry(loadPLYFileModel(filename.getPtr(), false));
                    } else if (ext == "obj") {
                        streamGeometry(loadObjFileModel(filename.getPtr()));
                    } else {
                        current_model_ = MODEL_EXAMPLE;
                        model_changed_ = true;
                    }
                } else {
                    current_model_ = MODEL_EXAMPLE;
                    model_changed_ = true;
                }
            } break;
            case SIMPLIFIED_MODEL_FROM_FILE: {
                // EXTRA: Load PLY and simplify.
                auto filename = window_.showFileLoadDialog("Load new mesh");
                if (filename.getLength()) {
                    auto ext = filename.substring(filename.lastIndexOf(".") + 1).toLower();
                    if (ext == "ply") {
                        streamGeometry(loadPLYFileModel(filename.getPtr(), true));
                    } else {
                        current_model_ = MODEL_EXAMPLE;
                        model_changed_ = true;
                    }
                } else {
                    current_model_ = MODEL_EXAMPLE;
                    model_changed_ = true;
                }
            } break;
            default:
                assert(false && "invalid model type");
        }
    }

    if (shading_mode_changed_) {
        common_ctrl_.message(shading_toggle_ ? "Directional light shading using vertex normals; direction to light (0.5, 0.5, -0.6)" : "High visibility shading, color from vertex ID");
        shading_mode_changed_ = false;
    }

    if (ev.type == Window::EventType_KeyDown) {
        // YOUR CODE HERE (R1)
        // React to user input and move the model.
        // Look in framework/gui/Keys.hpp for more key codes.
        // Visual Studio tip: you can right-click an identifier like FW_KEY_HOME
        // and "Go to definition" to jump directly to where the identifier is defined.
        if (ev.key == FW_KEY_HOME) camera_rotation_angle_ -= 0.05 * FW_PI;
        else if (ev.key == FW_KEY_END)
            camera_rotation_angle_ += 0.05 * FW_PI;
        else if (ev.key == FW_KEY_LEFT)
            this->object_transformation_matrix_(0, 3) -= 0.05;
        else if (ev.key == FW_KEY_RIGHT)
            this->object_transformation_matrix_(0, 3) += 0.05;
        else if (ev.key == FW_KEY_UP)
            this->object_transformation_matrix_(1, 3) += 0.05;
        else if (ev.key == FW_KEY_DOWN)
            this->object_transformation_matrix_(1, 3) -= 0.05;
        else if (ev.key == FW_KEY_A) {
            this->object_rotation_angle_ += 0.05 * FW_PI;
            this->update_rotation();
        } else if (ev.key == FW_KEY_D) {
            this->object_rotation_angle_ -= 0.05 * FW_PI;
            this->update_rotation();
        } else if (ev.key == FW_KEY_Q) {
            float prev_scale{this->object_x_scale_};
            this->object_x_scale_ += 0.05;
            this->update_scale(prev_scale);
        } else if (ev.key == FW_KEY_E) {
            float prev_scale{this->object_x_scale_};
            this->object_x_scale_ -= 0.05;
            this->update_scale(prev_scale);
        } else if (ev.key == FW_KEY_WHEEL_UP) {
            this->camera_z_translation_ += 0.1;
        } else if (ev.key == FW_KEY_WHEEL_DOWN) {
            this->camera_z_translation_ -= 0.1;
        } else if (ev.key == FW_KEY_R) {
            if (this->animating_) {
                this->animating_ = false;
                this->timer_.end();
            } else {
                this->animating_ = true;
                this->timer_.start();
            }
            this->prev_time_ = 0.0;
        } else if (ev.key == FW_KEY_Z) {
            this->fov_ += 0.05 * FW_PI;
        } else if (ev.key == FW_KEY_C) {
            this->fov_ -= 0.05 * FW_PI;
        }
    }

    if (ev.type == Window::EventType_KeyUp) {}

    if (ev.type == Window::EventType_Mouse) {
        // EXTRA: you can put your mouse controls here.
        // Event::mouseDelta gives the distance the mouse has moved.
        // Event::mouseDragging tells whether some mouse buttons are currently down.
        // If you want to know which ones, you have to keep track of the button down/up events
        // (e.g. FW_KEY_MOUSE_LEFT).
        if (ev.mouseDragging) {
            this->camera_rotation_angle_ += ev.mouseDelta[0] * 0.01;
            this->camera_x_rotation_angle_ += ev.mouseDelta[1] * 0.01;
        }
    }

    if (ev.type == Window::EventType_Close) {
        window_.showModalMessage("Exiting...");
        delete this;
        return true;
    }

    window_.setVisible(true);
    if (ev.type == Window::EventType_Paint) render();

    window_.repaint();

    return false;
}

void App::update_scale(float prev_scale) { this->object_transformation_matrix_(0, 0) = this->object_transformation_matrix_(0, 0) / prev_scale * this->object_x_scale_; }

void App::update_rotation() {
    Mat3f rot = Mat3f::rotation(Vec3f(0, 1, 0), -this->object_rotation_angle_);
    this->object_transformation_matrix_.setCol(0, Vec4f(rot.getCol(0), 0));
    this->object_transformation_matrix_.setCol(1, Vec4f(rot.getCol(1), 0));
    this->object_transformation_matrix_.setCol(2, Vec4f(rot.getCol(2), 0));
    /**
	 * cos(\theta)	0	sin(\theta)
	 * 0			1	0
	 * -sin(\theta)	0	cos(\theta)
	 **/
    // this->current_transformation_(0, 0) = FW::cos(this->object_rotation_angle_);
    // this->current_transformation_(0, 2) = FW::sin(this->object_rotation_angle_);
    // this->current_transformation_(0, 2) = -FW::sin(this->object_rotation_angle_);
    // this->current_transformation_(2, 2) = FW::cos(this->object_rotation_angle_);
    this->object_transformation_matrix_(0, 0) *= this->object_x_scale_;
}

void App::initRendering() {
    // Ask the Nvidia framework for the GLContext object associated with the window.
    // As a side effect, this initializes the OpenGL context and lets us call GL functions.
    auto ctx = window_.getGL();

    // Create vertex attribute objects and buffers for vertex data.
    glGenVertexArrays(1, &gl_.static_vao);
    glGenVertexArrays(1, &gl_.dynamic_vao);
    glGenBuffers(1, &gl_.static_vertex_buffer);
    glGenBuffers(1, &gl_.dynamic_vertex_buffer);

    // Set up vertex attribute object for static data.
    glBindVertexArray(gl_.static_vao);
    glBindBuffer(GL_ARRAY_BUFFER, gl_.static_vertex_buffer);
    glEnableVertexAttribArray(ATTRIB_POSITION);
    glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(ATTRIB_NORMAL);
    glVertexAttribPointer(ATTRIB_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) offsetof(Vertex, normal));

    // Load the static data to the GPU; needs to be done only once.
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * SIZEOF_ARRAY(reference_plane_data), reference_plane_data, GL_STATIC_DRAW);

    // Set up vertex attribute object for dynamic data. We'll load the actual data later, whenever the model changes.
    glBindVertexArray(gl_.dynamic_vao);
    glBindBuffer(GL_ARRAY_BUFFER, gl_.dynamic_vertex_buffer);
    glEnableVertexAttribArray(ATTRIB_POSITION);
    glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(ATTRIB_NORMAL);
    glVertexAttribPointer(ATTRIB_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) offsetof(Vertex, normal));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Compile and link the shader program.
    // We use the Nvidia FW for creating the program; it's not difficult to do manually,
    // but takes about half a page of OpenGL boilerplate code.
    // This shader program will be used to draw everything except the user interface.
    // It consists of one vertex shader and one fragment shader.
    auto shader_program = new GLContext::Program(
        "#version 330\n" FW_GL_SHADER_SOURCE(
            layout(location = 0) in vec4 aPosition;
            layout(location = 1) in vec3 aNormal;

            out vec4 vColor;

            uniform mat4 uModelToWorld;
            uniform mat4 uWorldToClip;

            uniform mat4 aNormalTransformation;

            uniform float uShading;

            const vec3 distinctColors[6] = vec3[6](
                vec3(0, 0, 1), vec3(0, 1, 0), vec3(0, 1, 1),
                vec3(1, 0, 0), vec3(1, 0, 1), vec3(1, 1, 0));
            const vec3 directionToLight = normalize(vec3(0.5, 0.5, -0.6));

            void main() {
                // EXTRA: oops, someone forgot to transform normals here...

                // Note: Do not use uWorldToClip here!
                // vec3 fixedNormal = normalize((transpose(inverse(uModelToWorld)) * vec4(aNormal, 1.0)).xyz);
                vec3 fixedNormal = normalize((aNormalTransformation * vec4(aNormal, 1.0)).xyz);

                float clampedCosine = clamp(dot(fixedNormal, directionToLight), 0.0, 1.0);
                vec3 litColor = vec3(clampedCosine);
                vec3 generatedColor = distinctColors[gl_VertexID % 6];
                // gl_Position is a built-in output variable that marks the final position
                // of the vertex in clip space. Vertex shaders must write in it.
                gl_Position = uWorldToClip * uModelToWorld * aPosition;
                vColor = vec4(mix(generatedColor, litColor, uShading), 1);
            }),
        "#version 330\n" FW_GL_SHADER_SOURCE(
            in vec4 vColor;
            out vec4 fColor;
            void main() {
                fColor = vColor;
            }));
    // Tell the FW about the program so it gets properly destroyed at exit.
    ctx->setProgram("shaders", shader_program);

    // Get the IDs of the shader program and its uniform input locations from OpenGL.
    gl_.shader_program = shader_program->getHandle();
    gl_.world_to_clip_uniform = glGetUniformLocation(gl_.shader_program, "uWorldToClip");
    gl_.model_to_world_uniform = glGetUniformLocation(gl_.shader_program, "uModelToWorld");
    gl_.shading_toggle_uniform = glGetUniformLocation(gl_.shader_program, "uShading");
    gl_.normal_transformation_uniform = glGetUniformLocation(gl_.shader_program, "aNormalTransformation");
}

void App::render() {
    // Clear screen.
    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Extra: Easy viewport correction
    Vec2i window_size = this->window_.getSize();
    // int size = FW::min(window_size);
    // glViewport((window_size[0] - size) / 2, (window_size[1] - size) / 2, size, size);

    // Enable depth testing.
    glEnable(GL_DEPTH_TEST);

    // Set up a matrix to transform from world space to clip space.
    // Clip space is a [-1, 1]^3 space where OpenGL expects things to be
    // when it starts drawing them.

    // Our camera is aimed at origin, and orbits around origin at fixed distance.
    static const float camera_distance = 2.1f;
    Mat4f C;
    Mat3f rot = Mat3f::rotation(Vec3f(1, 0, 0), -this->camera_x_rotation_angle_) * Mat3f::rotation(Vec3f(0, 1, 0), -camera_rotation_angle_);
    C.setCol(0, Vec4f(rot.getCol(0), 0));
    C.setCol(1, Vec4f(rot.getCol(1), 0));
    C.setCol(2, Vec4f(rot.getCol(2), 0));
    C.setCol(3, Vec4f(0, 0, camera_distance + this->camera_z_translation_, 1));

    // Simple perspective.
    static const float fnear = 0.1f, ffar = 4.0f;
    Mat4f P;
    // Resources:
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/building-basic-perspective-projection-matrix
    // http://learnwebgl.brown37.net/08_projections/projections_perspective.html
    float s = 1.0 / tan(this->fov_ / 2.0);
    float aspect = (window_size[0] * 1.0) / (window_size[1] * 1.0);
    P.setCol(0, Vec4f(s, 0, 0, 0));
    P.setCol(1, Vec4f(0, s * aspect, 0, 0));

    P.setCol(2, Vec4f(0, 0, (ffar + fnear) / (ffar - fnear), 1));
    P.setCol(3, Vec4f(0, 0, -2 * ffar * fnear / (ffar - fnear), 0));

    Mat4f world_to_clip = P * C;

    // Set active shader program.
    glUseProgram(gl_.shader_program);
    glUniform1f(gl_.shading_toggle_uniform, shading_toggle_ ? 1.0f : 0.0f);
    glUniformMatrix4fv(gl_.world_to_clip_uniform, 1, GL_FALSE, world_to_clip.getPtr());

    // Draw the reference plane. It is already in world coordinates.
    auto identity = Mat4f();
    glUniformMatrix4fv(gl_.model_to_world_uniform, 1, GL_FALSE, identity.getPtr());
    glUniformMatrix4fv(gl_.normal_transformation_uniform, 1, GL_FALSE, identity.getPtr());
    glBindVertexArray(gl_.static_vao);
    glDrawArrays(GL_TRIANGLES, 0, SIZEOF_ARRAY(reference_plane_data));

    // YOUR CODE HERE (R1)
    // Set the model space -> world space transform to translate the model according to user input.
    Mat4f modelToWorld(this->object_transformation_matrix_);

    // Draw the model with your model-to-world transformation.
    // Note: Do not use world_to_clip here!
    Mat4f normal_transformation = invert(transpose(this->object_transformation_matrix_));
    glUniformMatrix4fv(gl_.normal_transformation_uniform, 1, GL_FALSE, normal_transformation.getPtr());

    glUniformMatrix4fv(gl_.model_to_world_uniform, 1, GL_FALSE, modelToWorld.getPtr());
    glBindVertexArray(gl_.dynamic_vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertex_count_));

    // Undo our bindings.
    glBindVertexArray(0);
    glUseProgram(0);

    // Check for OpenGL errors.
    GLContext::checkErrors();

    // Show status messages. You may find it useful to show some debug information in a message.
    common_ctrl_.message(sprintf("Use Home/End to rotate camera. Use A/D to rotate object. Use Q/E to scale object.\nUse Z/C to change FOV. Use R to animate."), "instructions");
    common_ctrl_.message(sprintf("Camera is at (%.2f %.2f %.2f) looking towards origin.",
                                 -sin(camera_rotation_angle_) * camera_distance, 0.0f,
                                 -cos(camera_rotation_angle_) * camera_distance),
                         "camerainfo");
}

vector<Vertex> App::loadPLYFileModel(string filename, bool simplify) {
    // http://paulbourke.net/dataformats/ply/
    // We assume that the input mesh is a pure triangle mesh.

    window_.showModalMessage(sprintf("Loading mesh from '%s'...", filename.c_str()));

    vector<Vec3f> positions, normals;
    vector<array<unsigned, 6>> faces;
    vector<array<unsigned, 3>> temp_faces;

    // Open input file stream for reading.
    ifstream input(filename, ios::in);

    int state = 0; // 0: in header | 1: in vertex list | 2: in face list
    int vertex_count, face_count;
    int counter = 0;

    // Read the file line by line.
    string line;

    bool failed = false;

    // Temporary objects to read data into.
    Vec3f v;
    string s;
    std::vector<std::string> items(3);
    array<unsigned, 6> f;      // Face index array
    array<unsigned, 3> temp_f; // Face index array

    while (getline(input, line)) {
        // Create a stream from the string to pick out one value at a time.
        istringstream iss(line);
        // Read the first token from the line into string 's'.
        iss >> s;
        // Read header
        if (state == 0) {
            if (s == "format") {
                iss >> s;
                if (s != "ascii") {
                    // Only ascii supported.
                    failed = true;
                    break;
                }
            }
            if (s == "element") {
                iss >> s;
                if (s == "vertex") {
                    iss >> vertex_count;
                } else if (s == "face") {
                    iss >> face_count;
                }
            }
            if (s == "end_header") { state = 1; }
            continue;
        }
        int loc = 0;
        // Read line contents.
        while (true) {
            if (s == "{") { break; }
            if ((loc + 1) > items.size()) {
                items.push_back(s);
            } else {
                items[loc] = s;
            }
            loc++;
            if (iss.eof()) { break; }
            iss >> s;
        }
        // Put line contents into appropriate vertex or face.
        if (state == 1) {
            for (size_t i = 0; i < 3; i++) { v[i] = std::stof(items[i]); }
            positions.push_back(v);
            counter++;
            if (counter == vertex_count) {
                state = 2;
                counter = 0;
            }
        } else if (state == 2) {
            // Ignore the first item
            for (size_t i = 1; i < 4; i++) {
                f[2 * (i - 1)] = std::stoi(items[i]);
                temp_f[i - 1] = std::stoi(items[i]);
                f[2 * (i - 1) + 1] = counter;
            }
            faces.push_back(f);
            temp_faces.push_back(temp_f);
            normals.push_back(normalize(cross(positions[f[0]] - positions[f[2]], positions[f[0]] - positions[f[4]])));
            counter++;
            if (counter == face_count) { break; }
        }
    }

    if (simplify) {
        auto res = this->simplifyMesh(positions, temp_faces);
        positions = get<0>(res);
        normals = get<1>(res);
        faces = get<2>(res);
    }


    if (failed) { common_ctrl_.message("Only ascii PLY format supported"); }

    common_ctrl_.message(("Loaded mesh from " + filename).c_str());
    return unpackIndexedData(positions, normals, faces);
}

std::tuple<unsigned, unsigned> get_edge_key(unsigned first_vertex_index, unsigned second_vertex_index) { return std::make_tuple(std::min(first_vertex_index, second_vertex_index), std::max(first_vertex_index, second_vertex_index)); }

std::tuple<float, Vec4f> calculate_cost_and_optimal_point(Quadric quadric) {
    auto modified_quadric(quadric);
    // Set last row
    modified_quadric(3, 0) = 0.0;
    modified_quadric(3, 1) = 0.0;
    modified_quadric(3, 2) = 0.0;
    modified_quadric(3, 3) = 1.0;

    auto optimal_point = invert(modified_quadric) * Vec4f(0.0, 0.0, 0.0, 1.0);
    // Oh GOD! Why can't I transpose a vector? I hate FW!
    // auto cost = FW::transpose(optimal_point) * quadric * optimal_point;
    auto cost = quadric(0, 0) * (optimal_point.x * optimal_point.x) +
                2 * quadric(0, 1) * (optimal_point.x * optimal_point.y) +
                2 * quadric(0, 2) * (optimal_point.x * optimal_point.z) +
                2 * quadric(0, 3) * optimal_point.x +
                quadric(1, 1) * (optimal_point.y * optimal_point.y) +
                2 * quadric(1, 2) * (optimal_point.y * optimal_point.z) +
                2 * quadric(1, 3) * optimal_point.y +
                quadric(2, 2) * (optimal_point.z * optimal_point.z) +
                2 * quadric(2, 3) * optimal_point.z +
                quadric(3, 3);
    return std::make_tuple(cost, optimal_point);
}


tuple<std::vector<Vec3f>, std::vector<Vec3f>, std::vector<std::array<unsigned, 6>>> App::simplifyMesh(std::vector<Vec3f> positions, std::vector<std::array<unsigned, 3>> faces) {
    // Note: We should check some conditions before and after an edge collapse.
    // We don't do that here! This is a very basic implementation.
    std::vector<Quadric> quadrics;
    quadrics.reserve(positions.size());
    // This would be simplified if we had a halfedge data structure.
    std::map<unsigned, std::tuple<unsigned, unsigned>> edge_index_to_key;
    std::map<std::tuple<unsigned, unsigned>, unsigned> edge_key_to_index;
    std::vector<Vec4f> optimal_points;
    std::map<unsigned, std::vector<unsigned>> vertex_to_faces;
    std::map<unsigned, std::vector<unsigned>> edge_to_faces;
    std::set<unsigned> dead_faces;
    std::set<unsigned> dead_vertices;
    unsigned face_counter = 0;
    unsigned edge_counter = 0;
    for (const auto &face_vertices : faces) {
        for (size_t i = 0; i < 3; i++) {
            vertex_to_faces[face_vertices[i]].push_back(face_counter);
            auto edge_key = get_edge_key(face_vertices[i], face_vertices[(i + 1) % 3]);
            if (!edge_key_to_index.count(edge_key)) {
                // New edge
                edge_index_to_key[edge_counter] = edge_key;
                edge_key_to_index[edge_key] = edge_counter;
                edge_counter++;
            }
            edge_to_faces[edge_key_to_index[edge_key]].push_back(face_counter);
        }
        face_counter++;
    }
    unsigned vertex_counter = 0;
    for (const auto &vertex_position : positions) {
        Quadric vertex_quadric;
        unsigned v0_index = vertex_counter;
        for (const auto &face_index : vertex_to_faces[vertex_counter]) {
            std::vector<unsigned> neighbors;
            auto face_vertices = faces[face_index];
            for (size_t i = 0; i < 3; i++) {
                if (face_vertices[i] != v0_index) { neighbors.push_back(face_vertices[i]); }
            }
            // C++ 17
            // auto [v1_index, v2_index] = neighbors;
            auto v1_index = neighbors[0], v2_index = neighbors[1];
            const auto &p0 = positions[v0_index], &p1 = positions[v1_index], &p2 = positions[v2_index];
            // These two vectors are in the plane
            auto v1 = p2 - p0;
            auto v2 = p1 - p0;
            // The cross product is a vector normal to the plane
            auto normal = normalize(cross(v1, v2));
            auto a = normal[0], b = normal[1], c = normal[2];
            // This evaluates a * x3 + b * y3 + c * z3 which equals -d
            auto d = -dot(normal, p2);
            // Vec4f face_equation{a, b, c, d};
            Quadric fundamental_error_quadric;
            fundamental_error_quadric.setRow(0, Vec4f(a * a, a * b, a * c, a * d));
            fundamental_error_quadric.setRow(1, Vec4f(a * b, b * b, b * c, b * d));
            fundamental_error_quadric.setRow(2, Vec4f(a * c, b * c, c * c, c * d));
            fundamental_error_quadric.setRow(3, Vec4f(a * d, b * d, c * d, d * d));
            vertex_quadric += fundamental_error_quadric;
        }
        quadrics.push_back(vertex_quadric);
        vertex_counter++;
    }

    // Compute costs and add the edges to the priority queue.
    // This is an updatable priority queue.
    better_priority_queue::updatable_priority_queue<unsigned, float> pq;
    for (const auto &item : edge_index_to_key) {
        auto edge_index = item.first;
        auto edge_vertices = item.second;
        auto quadric = quadrics[get<0>(edge_vertices)] + quadrics[get<1>(edge_vertices)];
        auto res = calculate_cost_and_optimal_point(quadric);
        auto cost = get<0>(res);
        auto optimal_point = get<1>(res);
        optimal_points.push_back(optimal_point);
        pq.push(edge_index, -cost);
    }

    int face_count = faces.size();
    // Choose the target_face_count.
    int target_face_count = std::max(face_count - 100, 100);
    while (face_count > target_face_count) {
        if (pq.empty()) { break; }
        auto popped = pq.pop_value();
        // Edge to be collapsed.
        auto edge_index = popped.key;
        auto optimal_point = optimal_points[edge_index];
        auto edge_key = edge_index_to_key[edge_index];
        auto first_vertex_index = get<0>(edge_key);
        auto second_vertex_index = get<1>(edge_key);
        auto faces_indices = edge_to_faces[edge_index];
        // Check if edge already removed.
        bool corrupted = false;
        for (const auto &item : faces_indices) {
            if (dead_faces.count(item)) {
                corrupted = true;
                break;
            }
        }
        if (corrupted || dead_vertices.count(first_vertex_index) || dead_vertices.count(second_vertex_index)) { continue; }
        // Move the first vertex to the optimal_point.
        positions[first_vertex_index] = optimal_point.getXYZ();
        // Update the quadric of the first vertex.
        quadrics[first_vertex_index] += quadrics[second_vertex_index];

        // Eliminate faces adjacent to the edge.
        for (const auto &item : faces_indices) { dead_faces.insert(item); }
        // Eliminate the second vertex. We keep the first vertex.
        dead_vertices.insert(second_vertex_index);
        // Update stuff used to move around the mesh.
        for (const auto &face_index : vertex_to_faces[second_vertex_index]) {
            // If face is dead, don't bother. The newly deleted faces are also in the dead_faces.
            if (dead_faces.count(face_index)) { continue; }
            // Replace the second vertex by the first one.
            int found_index;
            for (int i = 0; i < 3; i++) {
                if (faces[face_index][i] == second_vertex_index) {
                    found_index = i;
                    break;
                }
            }
            // Update edges that were connected to the second vertex.
            auto edge_key = get_edge_key(second_vertex_index, faces[face_index][(found_index + 1) % 3]);
            edge_index_to_key[edge_key_to_index[edge_key]] = get_edge_key(first_vertex_index, faces[face_index][(found_index + 1) % 3]);
            edge_key_to_index[get_edge_key(first_vertex_index, faces[face_index][(found_index + 1) % 3])] = edge_key_to_index[edge_key];
            edge_key = get_edge_key(faces[face_index][(found_index + 2) % 3], second_vertex_index);
            edge_index_to_key[edge_key_to_index[edge_key]] = get_edge_key(faces[face_index][(found_index + 2) % 3], first_vertex_index);
            edge_key_to_index[get_edge_key(faces[face_index][(found_index + 2) % 3], first_vertex_index)] = edge_key_to_index[edge_key];

            // Update faces that were connected to the second vertex.
            faces[face_index][found_index] = first_vertex_index;
            // Update vertex_face_map for the first vertex.
            vertex_to_faces[first_vertex_index].push_back(face_index);
        }
        face_count -= 2;

        for (const auto &face_index : vertex_to_faces[first_vertex_index]) {
            // If face is dead, don't bother.
            if (dead_faces.count(face_index)) { continue; }
            for (size_t i = 0; i < 3; i++) {
                auto edge_key = get_edge_key(faces[face_index][i], faces[face_index][(i + 1) % 3]);
                auto edge_index = edge_key_to_index[edge_key];
                auto quadric = quadrics[get<0>(edge_key)] + quadrics[get<1>(edge_key)];
                auto res = calculate_cost_and_optimal_point(quadric);
                auto cost = get<0>(res);
                auto optimal_point = get<1>(res);
                optimal_points[edge_index] = optimal_point;
                pq.update(edge_index, -cost);
            }
        }
    }
    // Deletion phase done.
    // Wrap things up and send them back!
    std::map<unsigned, unsigned> old_to_new;
    std::vector<Vec3f> new_positions, new_normals;
    std::vector<std::array<unsigned, 6>> new_faces;
    vertex_counter = 0;
    for (size_t i = 0; i < positions.size(); i++) {
        if (dead_vertices.count(i)) { continue; }
        old_to_new[i] = vertex_counter;
        new_positions.push_back(positions[i]);
        vertex_counter++;
    }
    std::array<unsigned, 3> new_face_vertices;
    std::array<unsigned, 6> new_face_item;
    for (size_t face_index = 0; face_index < faces.size(); face_index++) {
        if (dead_faces.count(face_index)) { continue; }
        for (size_t j = 0; j < 3; j++) { new_face_vertices[j] = old_to_new[faces[face_index][j]]; }
        auto normal = normalize(cross(new_positions[new_face_vertices[0]] - positions[new_face_vertices[1]], positions[new_face_vertices[0]] - positions[new_face_vertices[2]]));
        new_normals.push_back(normal);
        new_face_item[0] = new_face_vertices[0];
        new_face_item[2] = new_face_vertices[1];
        new_face_item[4] = new_face_vertices[2];
        new_face_item[1] = new_normals.size() - 1;
        new_face_item[3] = new_normals.size() - 1;
        new_face_item[5] = new_normals.size() - 1;
        new_faces.push_back(new_face_item);
    }
    return make_tuple(new_positions, new_normals, new_faces);
}

vector<Vertex> App::loadObjFileModel(string filename) {
    window_.showModalMessage(sprintf("Loading mesh from '%s'...", filename.c_str()));

    vector<Vec3f> positions, normals;
    vector<array<unsigned, 6>> faces;

    // Open input file stream for reading.
    ifstream input(filename, ios::in);

    // Read the file line by line.
    string line;
    while (getline(input, line)) {
        // Replace any '/' characters with spaces ' ' so that all of the
        // values we wish to read are separated with whitespace.
        for (auto &c : line)
            if (c == '/') c = ' ';

        // Temporary objects to read data into.
        array<unsigned, 6> f; // Face index array
        Vec3f v;
        string s;

        // Create a stream from the string to pick out one value at a time.
        istringstream iss(line);

        // Read the first token from the line into string 's'.
        // It identifies the type of object (vertex or normal or ...)
        iss >> s;

        if (s == "v") {
            // vertex position
            // YOUR CODE HERE (R4)
            // Read the three vertex coordinates (x, y, z) into 'v'.
            // Store a copy of 'v' in 'positions'.
            // See std::vector documentation for push_back.
            iss >> v.x >> v.y >> v.z;
            positions.push_back(v);
        } else if (s == "vn") {
            // normal
            // YOUR CODE HERE (R4)
            // Similar to above.
            iss >> v.x >> v.y >> v.z;
            normals.push_back(v);
        } else if (s == "f") {
            // face
            // YOUR CODE HERE (R4)
            // Read the indices representing a face and store it in 'faces'.
            // The data in the file is in the format
            // f v1/vt1/vn1 v2/vt2/vn2 ...
            // where vi = vertex index, vti = texture index, vni = normal index.
            //
            // Remember we already replaced the '/' characters with whitespaces.
            //
            // Since we are not using textures in this exercise, you can ignore
            // the texture indices by reading them into a temporary variable.

            unsigned sink; // Temporary variable for reading the unused texture indices.
            for (size_t i = 0; i < 3; i++) {
                iss >> f[2 * i];
                iss >> sink;
                iss >> f[2 * i + 1];
            }
            for (size_t i = 0; i < 6; i++) { f[i] -= 1; }

            faces.push_back(f);
            // Note that in C++ we index things starting from 0, but face indices in OBJ format start from 1.
            // If you don't adjust for that, you'll index past the range of your vectors and get a crash.

            // It might be a good idea to print the indices to see that they were read correctly.
            // cout << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << " " << f[4] << " " << f[5] << endl;
        }
    }
    common_ctrl_.message(("Loaded mesh from " + filename).c_str());
    return unpackIndexedData(positions, normals, faces);
}

void FW::init(void) { new App; }
