//
// Created by Samuel Ellgass on 4/12/22.
//

#ifndef CLOTHSIM_SANDSIMULATOR_H
#define CLOTHSIM_SANDSIMULATOR_H

#include <nanogui/nanogui.h>
#include <memory>

#include "camera.h"
#include "collision/collisionObject.h"
#include "sandbox.h"
#include "sand_particle.h"
#include "wind_field.h"

// TODO : TEMP IF NO BUILD
struct UserShader;
enum ShaderTypeHint { WIREFRAME = 0, NORMALS = 1, PHONG = 2 };

using namespace nanogui;

class sandSimulator {
public:
    sandSimulator(std::string project_root, Screen *screen, int framerate, int sim_steps, bool is_recording);
    ~sandSimulator();

    void init();

    void loadSandbox(Sandbox *sandbox);
    void loadSandparameters(SandParameters *sp);
    void loadCollisionObjects(vector<CollisionObject *> *objects);
    void loadWindFields(vector<wind_field *> *wind_fields);
    virtual bool isAlive();
    virtual void drawContents();

    // Screen events

    virtual bool cursorPosCallbackEvent(double x, double y);
    virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
    virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
    virtual bool dropCallbackEvent(int count, const char **filenames);
    virtual bool scrollCallbackEvent(double x, double y);
    virtual bool resizeCallbackEvent(int width, int height);

    // For offscreen rendering
    bool isPaused();
    void pause();
    void resume();
    int getFPS();

private:
    virtual void initGUI(Screen *screen);
    void drawWireframe(GLShader &shader);
    void drawNormals(GLShader &shader);
    void drawPhong(GLShader &shader);

    void load_shaders();
    void load_textures();

    // File management

    std::string m_project_root;

    // Camera methods

    virtual void resetCamera();
    virtual Matrix4f getProjectionMatrix();
    virtual Matrix4f getViewMatrix();
    virtual Matrix4f getSkyboxViewMatrix();

    // Default simulation values

    int frames_per_sec = 90;
    int simulation_steps = 30;
    bool is_recording = false;

    CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
    nanogui::Color color = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);

    Sandbox *sandbox;
    SandParameters *sp;
    vector<CollisionObject *> *collision_objects;
    vector<wind_field *> *wind_fields;

    // OpenGL attributes

    // TODO: USING PHONG BY DEFAULT

    int active_shader_idx = 2;

    vector<UserShader> shaders;
    GLShader skybox;
    vector<std::string> shaders_combobox_names;

    // OpenGL textures

    Vector3D m_gl_texture_1_size;
    Vector3D m_gl_texture_2_size;
    Vector3D m_gl_texture_3_size;
    Vector3D m_gl_texture_4_size;
    GLuint m_gl_texture_1;
    GLuint m_gl_texture_2;
    GLuint m_gl_texture_3;
    GLuint m_gl_texture_4;
    GLuint m_gl_cubemap_tex;

    // OpenGL customizable inputs

    double m_normal_scaling = 2.0;
    double m_height_scaling = 0.1;

    // Camera attributes

    CGL::Camera camera;
    CGL::Camera skyboxcamera;
    CGL::Camera canonicalCamera;

    double view_distance;
    double canonical_view_distance;
    double min_view_distance;
    double max_view_distance;

    double scroll_rate;

    // Screen methods

    Screen *screen;
    void mouseLeftDragged(double x, double y);
    void mouseRightDragged(double x, double y);
    void mouseMoved(double x, double y);

    // Mouse flags

    bool left_down = false;
    bool right_down = false;
    bool middle_down = false;

    // Keyboard flags

    bool ctrl_down = false;

    // Simulation flags

    bool is_paused = true;

    // Screen attributes

    int mouse_x;
    int mouse_y;

    int screen_w;
    int screen_h;

    bool is_alive = true;

    Vector2i default_window_size = Vector2i(1024, 800);
};

//TODO: TEMP IF NO BUILD
struct UserShader {
    UserShader(std::string display_name, std::shared_ptr<GLShader> nanogui_shader, ShaderTypeHint type_hint)
            : display_name(display_name)
            , nanogui_shader(nanogui_shader)
            , type_hint(type_hint) {
    }

    std::shared_ptr<GLShader> nanogui_shader;
    std::string display_name;
    ShaderTypeHint type_hint;

};


#endif //CLOTHSIM_SANDSIMULATOR_H
