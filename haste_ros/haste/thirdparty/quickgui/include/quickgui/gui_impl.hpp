// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace quickgui {

auto Gui::lookAt(const Vector3& position, const Vector3& target, const Vector3& up) -> Quaternion {
  Eigen::Matrix<Scalar, 3, 3> R;
  R.col(2) = (target - position).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));
  return Quaternion{R};
}

auto Gui::DrawPlanarRectangle() -> void {
  glBegin(GL_LINE_LOOP);
  glVertex2f(-0.5, -0.5);
  glVertex2f(+0.5, -0.5);
  glVertex2f(+0.5, +0.5);
  glVertex2f(-0.5, +0.5);
  glEnd();
}

auto Gui::DrawPlanarRectangle(const Vector3& size, const Vector3& center) -> void {
  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(size.x(), size.y(), size.z());
  DrawPlanarRectangle();
  glPopMatrix();
}

auto Gui::DrawCube() -> void {
  glBegin(GL_QUADS);// Begin drawing the color cube with 6 quads
  // Top face (y = 1.0f)
  // Define vertices in counter-clockwise (CCW) order with normal pointing out
  glColor3f(0.0f, 1.0f, 0.0f);// Green
  glVertex3f(1.0f, 1.0f, -1.0f);
  glVertex3f(-1.0f, 1.0f, -1.0f);
  glVertex3f(-1.0f, 1.0f, 1.0f);
  glVertex3f(1.0f, 1.0f, 1.0f);

  // Bottom face (y = -1.0f)
  glColor3f(1.0f, 0.5f, 0.0f);// Orange
  glVertex3f(1.0f, -1.0f, 1.0f);
  glVertex3f(-1.0f, -1.0f, 1.0f);
  glVertex3f(-1.0f, -1.0f, -1.0f);
  glVertex3f(1.0f, -1.0f, -1.0f);

  // Front face  (z = 1.0f)
  glColor3f(1.0f, 0.0f, 0.0f);// Red
  glVertex3f(1.0f, 1.0f, 1.0f);
  glVertex3f(-1.0f, 1.0f, 1.0f);
  glVertex3f(-1.0f, -1.0f, 1.0f);
  glVertex3f(1.0f, -1.0f, 1.0f);

  // Back face (z = -1.0f)
  glColor3f(1.0f, 1.0f, 0.0f);// Yellow
  glVertex3f(1.0f, -1.0f, -1.0f);
  glVertex3f(-1.0f, -1.0f, -1.0f);
  glVertex3f(-1.0f, 1.0f, -1.0f);
  glVertex3f(1.0f, 1.0f, -1.0f);

  // Left face (x = -1.0f)
  glColor3f(0.0f, 0.0f, 1.0f);// Blue
  glVertex3f(-1.0f, 1.0f, 1.0f);
  glVertex3f(-1.0f, 1.0f, -1.0f);
  glVertex3f(-1.0f, -1.0f, -1.0f);
  glVertex3f(-1.0f, -1.0f, 1.0f);

  // Right face (x = 1.0f)
  glColor3f(1.0f, 0.0f, 1.0f);// Magenta
  glVertex3f(1.0f, 1.0f, -1.0f);
  glVertex3f(1.0f, 1.0f, 1.0f);
  glVertex3f(1.0f, -1.0f, 1.0f);
  glVertex3f(1.0f, -1.0f, -1.0f);
  glEnd();// End of drawing color-cube
}

Gui::Gui() { resetCamera(); }

Gui::~Gui() { destroyEnvironment_(); }

auto Gui::startInNewThread() -> Thread& {
  //  thread_ = std::thread(&Gui::run_, this);
  thread_ = std::thread([&]() {
    initializeEnvironment_();
    run_();
  });

  DLOG(INFO) << "Gui " << thread_.get_id() << " started by thread " << std::this_thread::get_id() << ".";
  return thread_;
}

auto Gui::start() -> void {
  initializeEnvironment_();
  run_();
}

auto Gui::run_() -> void {
  glEnable(GL_DEPTH_TEST);

  ImVec4 clear_color = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);

  // Main loop
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);

    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    static bool show_demo_window = true;
    if (show_demo_window) ImGui::ShowDemoWindow(&show_demo_window);

    // Rendering
    ImGui::Render();

    handleDeviceInput_();

    loadProjection_();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    DrawCube();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
  }
}

auto Gui::initializeEnvironment_() -> void {
  // glfw Initialization
  glfwSetErrorCallback([](int error, const char* description) {
    LOG(ERROR) << "Glfw Error " << error << ": " << std::string(description);
  });

  if (!glfwInit()) LOG(ERROR) << "Glfw could not be initialized";

  // OpenGL + GLSL version: GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

  // Create window with graphics context
  std::string window_name = "Window";
  window_ = glfwCreateWindow(1280, 720, window_name.c_str(), NULL, NULL);
  if (window_ == NULL) LOG(ERROR) << "Window could not be created";

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);// Enable vsync

  //  glfwSetCursorPosCallback(window_, mouse_callback);

  // Initialize OpenGL loader
  bool err = glewInit() != GLEW_OK;
  if (err) { LOG(ERROR) << "Failed to initialize OpenGL loader!\n"; }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  (void) io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;// Enable Keyboard Controls
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad
  // Controls
  io.WantCaptureKeyboard = true;
  io.WantCaptureMouse = true;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

auto Gui::destroyEnvironment_() -> void {
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_);
  glfwTerminate();
}

auto Gui::loadProjection_() -> void {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  int display_w, display_h;
  glfwGetFramebufferSize(window_, &display_w, &display_h);// TODO: Repeated

  gluPerspective(kViewFov_, float(display_w) / display_h, kMinClipDistance_, kMaxClipDistance_);

  glPushMatrix();

  Vector3 camera_front = camera_quat_.toRotationMatrix() * Vector3::UnitZ();
  Vector3 camera_up = Vector3::UnitY();

  gluLookAt(camera_pos_.x(), camera_pos_.y(), camera_pos_.z(), (camera_pos_ + camera_front).x(),
            (camera_pos_ + camera_front).y(), (camera_pos_ + camera_front).z(), camera_up.x(), camera_up.y(),
            camera_up.z());
}

auto Gui::handleDeviceInput_() -> void {
  Vector3 camera_z = camera_quat_.toRotationMatrix() * Vector3::UnitZ();
  Vector3 camera_up = Vector3::UnitY();
  Vector3 camera_x = camera_up.cross(camera_z);
  Vector3 camera_y = camera_x.cross(camera_z);

  auto frame_dt = 1.0 / ImGui::GetIO().Framerate;
  auto disp = kNavigationTranslationVelocity_ * frame_dt;

  if (ImGui::IsKeyDown(GLFW_KEY_W)) { translateCamera(camera_z * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_S)) { translateCamera(-camera_z * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_A)) { translateCamera(camera_x * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_D)) { translateCamera(-camera_x * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_Q)) { translateCamera(Vector3::UnitY() * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_E)) { translateCamera(-Vector3::UnitY() * disp); }
  if (ImGui::IsKeyDown(GLFW_KEY_R)) { resetCamera(); }

  static bool dragging_flag = false;

  //  camera_quat_ = lookAt(camera_pos_, Vector3::Zero(), Vector3::UnitY());

  if (!ImGui::IsWindowFocused(ImGuiFocusedFlags_AnyWindow)
      && ImGui::IsMouseDragging(ImGuiMouseButton_::ImGuiMouseButton_Left)) {
    static Eigen::Quaterniond root_quat;

    if (!dragging_flag) {
      dragging_flag = true;
      root_quat = camera_quat_;
    }

    auto delta = ImGui::GetMouseDragDelta();
    auto ang_disp_scale = kNavigationRotationVelocity_;

    camera_quat_ = Eigen::AngleAxis<Scalar>(-delta.x * ang_disp_scale, Vector3::UnitY()) * root_quat
        * Eigen::AngleAxis<Scalar>(delta.y * ang_disp_scale, Vector3::UnitX())

        ;
  } else {
    dragging_flag = false;
  }
}

auto Gui::resetCamera() -> void {
  camera_pos_.setZero();
  camera_quat_.setIdentity();
}

auto Gui::translateCamera(const Vector3& dt) -> void { camera_pos_ = camera_pos_ + dt; };

}// namespace quickgui