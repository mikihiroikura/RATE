// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glog/logging.h>

#include <Eigen/Geometry>
#include <thread>

#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "implot/implot.h"
#include "implot/implot_internal.h"

// TODO: remove unnecessary inlining.
// TODO: Naming formatting.
namespace quickgui {
class Gui {
 public:
  using Thread = std::thread;
  using Scalar = double;// TODO: add traits
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using WindowPtr = GLFWwindow*;
  using WindowName = std::string;
  using ColorMap = std::vector<std::array<GLfloat, 3>>;

  static constexpr GLfloat kColorBlack[] = {0.0f, 0.0f, 0.0f};
  static constexpr GLfloat kColorWhite[] = {1.0f, 1.0f, 1.0f};
  static constexpr GLfloat kColorRed[] = {1.0f, 0.0f, 0.0f};
  static constexpr GLfloat kColorBlue[] = {0.0f, 0.0f, 1.0f};
  static constexpr GLfloat kColorGreen[] = {0.0f, 1.0f, 0.0f};
  static constexpr GLfloat kColorMagenta[] = {1.0f, 0.0f, 1.0f};
  static constexpr GLfloat kColorCyan[] = {0.0f, 1.0f, 1.0f};

  static auto DrawCube() -> void;
  static auto lookAt(const Vector3& position, const Vector3& target, const Vector3& up) -> Quaternion;
  static auto DrawPlanarRectangle() -> void;
  static auto DrawPlanarRectangle(const Vector3& size, const Vector3& center = Vector3::Zero()) -> void;
  static auto RandomColorMap_(size_t n = 100) -> ColorMap;

  Gui();
  ~Gui();

  auto start() -> void;
  auto startInNewThread() -> Thread&;
  auto translateCamera(const Vector3& dt) -> void;
  virtual auto resetCamera() -> void;

 protected:
  static constexpr auto kViewFov_ = 60.0;
  static constexpr auto kMinClipDistance_ = 0.01;
  static constexpr auto kMaxClipDistance_ = 30;
  static constexpr auto kNavigationTranslationVelocity_ = 1.0;
  static constexpr auto kNavigationRotationVelocity_ = 0.005;

  auto loadProjection_() -> void;
  auto initializeEnvironment_() -> void;
  auto destroyEnvironment_() -> void;
  virtual auto handleDeviceInput_() -> void;
  virtual auto run_() -> void;

  Thread thread_;
  Vector3 camera_pos_;
  Quaternion camera_quat_;
  WindowPtr window_;
};
}// namespace quickgui

#include "gui_impl.hpp"