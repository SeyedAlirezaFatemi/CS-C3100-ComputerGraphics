#pragma once

#include "gui/CommonControls.hpp"
#include "gui/Window.hpp"

#include <string>
#include <tuple>
#include <vector>

namespace FW {
    using Quadric = Mat4f;

    struct Vertex {
        Vec3f position;
        Vec3f normal;
    };

    struct glGeneratedIndices {
        GLuint static_vao, dynamic_vao;
        GLuint shader_program;
        GLuint static_vertex_buffer, dynamic_vertex_buffer;
        GLuint model_to_world_uniform, world_to_clip_uniform, shading_toggle_uniform, normal_transformation_uniform;
    };

    class App : public Window::Listener {
    private:
        enum CurrentModel {
            MODEL_EXAMPLE,
            MODEL_USER_GENERATED,
            MODEL_FROM_INDEXED_DATA,
            MODEL_FROM_FILE,
            SIMPLIFIED_MODEL_FROM_FILE,
        };

    public:
        App();             // constructor
        ~App() override {} // destructor

        bool handleEvent(const Window::Event &ev) override;

    private:
        App(const App &);            // forbid copy
        App &operator=(const App &); // forbid assignment

        void initRendering();

        void render();

        std::vector<Vertex> loadObjFileModel(std::string filename);

        // EXTRA
        std::vector<Vertex> loadPLYFileModel(std::string filename, bool simplify);
        // EXTRA: Mesh Simplification
        std::tuple<std::vector<Vec3f>, std::vector<Vec3f>, std::vector<std::array<unsigned, 6>>> simplifyMesh(std::vector<Vec3f> positions, std::vector<std::array<unsigned, 3>> faces);

        void streamGeometry(const std::vector<Vertex> &vertices);

        void update_rotation();

        void update_scale(float prev_scale);

        Window window_;
        CommonControls common_ctrl_;

        CurrentModel current_model_;
        bool model_changed_;
        bool shading_toggle_;
        bool shading_mode_changed_;

        glGeneratedIndices gl_;

        size_t vertex_count_;

        float camera_rotation_angle_; // Rotation around the y axis

        // YOUR CODE HERE (R1)
        // Add a class member to store the current translation.
        Mat4f object_transformation_matrix_;
        float object_rotation_angle_;
        float object_x_scale_;

        float camera_z_translation_;
        float camera_x_rotation_angle_; // Rotation around the x axis

        // EXTRA:
        // For animation extra credit you can use the framework's Timer class.
        // The .start() and .unstart() methods start and stop the timer; when it's
        // running, .end() gives you seconds passed after last call to .end().
        Timer timer_;
        float prev_time_;
        bool animating_;

        // EXTRA: Viewport and perspective
        float fov_;
    };

} // namespace FW
