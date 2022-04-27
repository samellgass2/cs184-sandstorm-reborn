#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#else
#include <getopt.h>
#include <unistd.h>
#endif
#include <unordered_set>
#include <stdlib.h> // atoi for getopt inputs


#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "json.hpp"
#include "misc/file_utils.h"
#include "sandbox.h"
#include "sand_particle.h"
#include "sandSimulator.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[SandSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string SANDBOX = "sandbox";
const string WIND = "wind";
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, SANDBOX, WIND};

sandSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Sand Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -a     <INT>       Sphere vertices latitude direction.\n");
  printf("  -o     <INT>       Sphere vertices longitude direction.\n");
  printf("  -s     <STRING>  Video (.mov) file to save output to in windowless mode\n");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

bool loadObjectsFromFile(string filename, Sandbox *sandbox, SandParameters *sp, vector<CollisionObject *>* objects, vector<wind_field *> *wind_fields, int sphere_num_lat, int sphere_num_lon) {
  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();
//    cout << key.substr(0, 6) << endl;

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
//    if (query == VALID_KEYS.end()) {
//      cout << "Invalid scene object found: " << key << endl;
//      exit(-1);
//    }

    // Retrieve object
    json object = it.value();

    if (key.substr(0, 6) == SPHERE) {
      Vector3D origin;
      double friction, radius;

      auto it_origin = object.find("origin");
      if (it_origin != object.end()) {
        vector<double> vec_origin = *it_origin;
        origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
      } else {
        incompleteObjectError("sphere", "origin");
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;
      } else {
        incompleteObjectError("sphere", "radius");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("sphere", "friction");
      }

      Sphere *s = new Sphere(origin, radius, friction, sphere_num_lat, sphere_num_lon);
      objects->push_back(s);
    } else if (key.substr(0, 6) == PLANE) { // PLANE
      Vector3D point, normal;
      double friction, sand_radius, length, width;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      auto it_length = object.find("length");
      if (it_length != object.end()) {
        length = *it_length;
      } else {
        incompleteObjectError("plane", "length");
      }

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("plane", "width");
      }

      Plane *p = new Plane(point, normal, friction, length, width);
      objects->push_back(p);
    } else if (key.substr(0, 4) == WIND) {
      Vector3D top_left, bottom_right;
      vector<double> xcoefficients;
      vector<double> ycoefficients;
      vector<double> zcoefficients;
      double radius = 0;
      double a;
      double b;
      int num_coefficients;
      double magnitude = 0;


      auto it_top_left = object.find("top_left");
      if (it_top_left != object.end()) {
        vector<double> top_left_vec = *it_top_left;
        top_left = Vector3D(top_left_vec[0], top_left_vec[1], top_left_vec[2]);
      } else {
        incompleteObjectError("sandbox", "top_left");
      }

      auto it_bottom_right = object.find("bottom_right");
      if (it_bottom_right != object.end()) {
        vector<double> bottom_right_vec = *it_bottom_right;
        bottom_right = Vector3D(bottom_right_vec[0], bottom_right_vec[1], bottom_right_vec[2]);
      } else {
        incompleteObjectError("sandbox", "bottom_right");
      }

      auto it_coefs = object.find("num_coefficients");
      if (it_coefs != object.end()) {
        num_coefficients = *it_coefs;
      }
      //std::cout << num_coefficients << std::endl;

      auto it_xcoef = object.find("xcoefficients");
      if (it_xcoef != object.end()) {
        vector<double> x_placeholder = *it_xcoef;
        for (int i = 0; i < num_coefficients; i++) {
          xcoefficients.push_back(x_placeholder[i]);
          //std::cout << x_placeholder[i] << std::endl;
        }
      }

      auto it_ycoef = object.find("ycoefficients");
      if (it_ycoef != object.end()) {
        vector<double> y_placeholder = *it_ycoef;
        for (int i = 0; i < num_coefficients; i++) {
          ycoefficients.push_back(y_placeholder[i]);
          //std::cout << y_placeholder[i] << std::endl;
        }
      }

      auto it_zcoef = object.find("zcoefficients");
      if (it_xcoef != object.end()) {
        vector<double> z_placeholder = *it_zcoef;
        for (int i = 0; i < num_coefficients; i++) {
          zcoefficients.push_back(z_placeholder[i]);
          //std::cout << z_placeholder[i] << std::endl;
        }
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;

        auto it_mag = object.find("magnitude");
        if (it_mag != object.end()) {
          magnitude = *it_mag;
        }

        auto it_origin = object.find("center");
        if (it_origin != object.end()) {
          vector<double> origin_placeholder = *it_origin;
          a = origin_placeholder[0];
          b = origin_placeholder[1];

          wind_field *windField = new wind_field();
          windField->radius = radius;
          windField->a = a;
          windField->b = b;
          windField->magnitude = magnitude;
          windField->is_cyclone = true;
          wind_fields->push_back(windField);

        }


      } else {
        wind_field *windField = new wind_field(top_left, bottom_right, xcoefficients, ycoefficients, zcoefficients);
        wind_fields->push_back(windField);
      }


    } else {
      // SANDBOX
      Vector3D top_left, bottom_right;
      int num_sand_particles;
      double sand_radius, alpha, beta, k_d, k_r, mu, k_t, mass;

      auto it_top_left = object.find("top_left");
      if (it_top_left != object.end()) {
        vector<double> top_left_vec = *it_top_left;
        top_left = Vector3D(top_left_vec[0], top_left_vec[1], top_left_vec[2]);
      } else {
        incompleteObjectError("sandbox", "top_left");
      }

      auto it_bottom_right = object.find("bottom_right");
      if (it_bottom_right != object.end()) {
        vector<double> bottom_right_vec = *it_bottom_right;
        bottom_right = Vector3D(bottom_right_vec[0], bottom_right_vec[1], bottom_right_vec[2]);
      } else {
        incompleteObjectError("sandbox", "bottom_right");
      }

      auto it_num_sand_particles = object.find("num_sand_particles");
      if (it_num_sand_particles != object.end()) {
        num_sand_particles = *it_num_sand_particles;
      } else {
        incompleteObjectError("sandbox", "num_sand_particles");
      }

      auto it_sand_rad = object.find("sand_radius");
      if (it_sand_rad != object.end()) {
        sand_radius = *it_sand_rad;
      } else {
        incompleteObjectError("sandbox", "top_left");
      }

      auto it_sand_alpha = object.find("alpha");
      if (it_sand_alpha != object.end()) {
        alpha = *it_sand_alpha;
      } else {
        incompleteObjectError("sandbox", "alpha");
      }

      auto it_sand_beta = object.find("beta");
      if (it_sand_beta != object.end()) {
        beta = *it_sand_beta;
      } else {
        incompleteObjectError("sandbox", "beta");
      }

      auto it_sand_k_d = object.find("k_d");
      if (it_sand_k_d != object.end()) {
        k_d = *it_sand_k_d;
      } else {
        incompleteObjectError("sandbox", "k_d");
      }

      auto it_sand_k_r = object.find("k_r");
      if (it_sand_k_r != object.end()) {
        k_r = *it_sand_k_r;
      } else {
        incompleteObjectError("sandbox", "k_r");
      }

      auto it_sand_mu = object.find("mu");
      if (it_sand_mu != object.end()) {
        mu = *it_sand_mu;
      } else {
        incompleteObjectError("sandbox", "mu");
      }

      auto it_sand_k_t = object.find("k_t");
      if (it_sand_k_t != object.end()) {
        k_t = *it_sand_k_t;
      } else {
        incompleteObjectError("sandbox", "k_t");
      }

      auto it_sand_mass = object.find("mass");
      if (it_sand_k_t != object.end()) {
        mass = *it_sand_mass;
      } else {
        incompleteObjectError("sandbox", "mass");
      }

      sandbox->top_left = top_left;
      sandbox->bottom_right = bottom_right;
      sandbox->num_sand_particles = num_sand_particles;
      sandbox->sand_radius = sand_radius;
      sandbox->mu = mu;
      sp->alpha = alpha;
      sp->beta = beta;
      sp->k_r = k_r;
      sp->k_d = k_d;
      sp->k_t = k_t;
      sp->mass = mass;
    }
  }

  i.close();
  
  return true;
}

bool is_valid_project_root(const std::string& search_path) {
    std::stringstream ss;
    ss << search_path;
    ss << "/";
    ss << "shaders/Default.vert";
    
    return FileUtils::file_exists(ss.str());
}

// Attempt to locate the project root automatically
bool find_project_root(const std::vector<std::string>& search_paths, std::string& retval) {
  
  for (std::string search_path : search_paths) {
    if (is_valid_project_root(search_path)) {
      retval = search_path;
      return true;
    }
  }
  return false;
}

void saveImage(char* filepath, GLFWwindow* w, int frame) {
  int width, height;
  glfwGetFramebufferSize(w, &width, &height);
  GLsizei nrChannels = 3;
  GLsizei stride = nrChannels * width;
  stride += (stride % 4) ? (4 - stride % 4) : 0;
  GLsizei bufferSize = stride * height;
  std::vector<char> buffer(bufferSize);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  glReadBuffer(GL_FRONT);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
  stbi_flip_vertically_on_write(true);
  stbi_write_png(filepath, width, height, nrChannels, buffer.data(), stride);
}

int main(int argc, char **argv) {
  // Attempt to find project root
  std::vector<std::string> search_paths = {
    ".",
    "..",
    "../..",
    "../../.."
  };
  std::string project_root;
  bool found_project_root = find_project_root(search_paths, project_root);
  
  Sandbox sandbox;
  SandParameters sp;
  vector<CollisionObject *> objects;
  vector<wind_field *> wind_fields;

  std::cout << "Sandbox initialized" << std::endl;
  
  int c;
  
  int sphere_num_lat = 40;
  int sphere_num_lon = 40;
  
  std::string file_to_load_from;
  std::string file_to_save_to;
  bool file_specified = false;
  
  while ((c = getopt (argc, argv, "f:r:a:o:")) != -1) {
    switch (c) {
      case 'f': {
        file_to_load_from = optarg;
        file_specified = true;
        break;
      }
      case 'r': {
        project_root = optarg;
        if (!is_valid_project_root(project_root)) {
          std::cout << "Warn: Could not find required file \"shaders/Default.vert\" in specified project root: " << project_root << std::endl;
        }
        found_project_root = true;
        break;
      }
      case 'a': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lat = arg_int;
        break;
      }
      case 'o': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lon = arg_int;
        break;
      }
      case 's': {
        file_to_save_to = optarg;
        break;
      }
      default: {
        usageError(argv[0]);
        break;
      }
    }
  }
  
  if (!found_project_root) {
    std::cout << "Error: Could not find required file \"shaders/Default.vert\" anywhere!" << std::endl;
    return -1;
  } else {
    std::cout << "Loading files starting from: " << project_root << std::endl;
  }

  if (!file_specified) { // No arguments, default initialization
    std::stringstream def_fname;
    def_fname << project_root;
    def_fname << "/scene/pinned2.json";
    file_to_load_from = def_fname.str();
  }
  
  bool success = loadObjectsFromFile(file_to_load_from, &sandbox, &sp, &objects, &wind_fields, sphere_num_lat, sphere_num_lon);
  if (!success) {
    std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the sandbox object
  std::cout << sandbox.num_sand_particles << std::endl;

  sandbox.generate_particles();

  // Initialize the sandSimulator object
  app = new sandSimulator(project_root, screen);
  app->loadSandbox(&sandbox);
  app->loadSandparameters(&sp);
  app->loadCollisionObjects(&objects);
  app->loadWindFields(&wind_fields);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

//  int frame = 0;
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

//    saveImage(const_cast<char*>((std::to_string(frame) + ".png").c_str()), window, frame);
//    frame++;

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}
