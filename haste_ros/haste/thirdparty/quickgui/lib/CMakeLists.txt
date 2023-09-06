# Required CMake version and project name.
cmake_minimum_required(VERSION 3.0.0)
project(ImGuiPlot VERSION 1.0.0 LANGUAGES CXX)

# Find required packages.
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set(IMGUI_HEADERS
    imgui/imconfig.h
    imgui/imgui_internal.h
    imgui/imgui.h
    imgui/backends/imgui_impl_glfw.h
    imgui/backends/imgui_impl_opengl3.h
    )

set(IMGUI_SOURCES
    imgui/imgui_draw.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui.cpp
    imgui/imgui_tables.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
    )

set(IMPLOT_HEADERS,
    implot/implot.h
    implot/implot_internal.h
    )

set(IMPLOT_SOURCES
    implot/implot.cpp
    implot/implot_items.cpp
    implot/implot_demo.cpp
    )

include_directories(
    imgui
    implot
)

add_library(ImGui STATIC ${IMGUI_SOURCES} ${IMGUI_HEADERS})
target_link_libraries(ImGui ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw)
target_include_directories(ImGui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/imgui")


add_library(ImPlot STATIC ${IMPLOT_SOURCES} ${IMPLOT_HEADERS})
target_link_libraries(ImPlot ImGui)
target_include_directories(ImPlot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/implot")


