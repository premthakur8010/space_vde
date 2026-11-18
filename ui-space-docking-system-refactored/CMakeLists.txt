# ==============================================================================
# CMakeLists.txt
# Top-level CMake build configuration for the VDE Docking Simulator UI client.
# Resolves OpenGL, GLFW, and GLM dependencies, compiles vendored GLAD, Dear ImGui,
# and ImPlot static libraries, and builds the 120 Hz `vde_sim` executable.
# ==============================================================================

cmake_minimum_required(VERSION 3.20)

# Work around WinLibs/MinGW ld bug when GCC's realpath contains spaces behind C:/tools/mingw64.
if(WIN32 AND EXISTS "C:/tools/mingw64/x86_64-w64-mingw32/lib")
  string(APPEND CMAKE_C_FLAGS_INIT " -B C:/tools/mingw64/x86_64-w64-mingw32/lib")
  string(APPEND CMAKE_CXX_FLAGS_INIT " -B C:/tools/mingw64/x86_64-w64-mingw32/lib")
endif()

project(VdeDockingSimulator LANGUAGES C CXX)

# Enforce standard ISO C++20 across all targets without compiler-specific extensions.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform warning flags and Windows macro guards.
if(WIN32)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()
if(MSVC)
  add_compile_options(/utf-8 /W3)
else()
  add_compile_options(-Wall)
endif()

# ---------------------------------------------------------------- external dependencies
# Locate system OpenGL and fetch GLFW (windowing/input) and GLM (3D math) if not installed locally.
find_package(OpenGL REQUIRED)
include(FetchContent)

find_package(glfw3 CONFIG QUIET)
if(NOT glfw3_FOUND)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  if(UNIX AND NOT APPLE)
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
  endif()
  FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw)
endif()

find_package(glm CONFIG QUIET)
if(NOT glm_FOUND)
  set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glm)
endif()

# ---------------------------------------------------------------- vendored (third_party/)
# Build GLAD OpenGL 4.5 core loader from vendored C source.
add_library(glad STATIC third_party/glad/src/gl.c)
target_include_directories(glad PUBLIC third_party/glad/include)

# Build Dear ImGui (docking + multi-viewport backends) and ImPlot together as one static library.
add_library(imgui STATIC
  third_party/imgui/imgui.cpp
  third_party/imgui/imgui_draw.cpp
  third_party/imgui/imgui_tables.cpp
  third_party/imgui/imgui_widgets.cpp
  third_party/imgui/backends/imgui_impl_glfw.cpp
  third_party/imgui/backends/imgui_impl_opengl3.cpp
  third_party/implot/implot.cpp
  third_party/implot/implot_items.cpp)
target_include_directories(imgui PUBLIC
  third_party/imgui third_party/imgui/backends third_party/implot)
target_link_libraries(imgui PUBLIC glfw)

# ---------------------------------------------------------------- application
# Main visualization client executable built from the 5 application source files.
add_executable(vde_sim
  src/main.cpp
  src/Client.cpp
  src/Renderer.cpp
  src/UiCore.cpp
  src/UiWindows.cpp)

target_include_directories(vde_sim PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../space_vde/include
)
target_link_libraries(vde_sim PRIVATE
  glad imgui glm::glm glfw OpenGL::GL
  $<$<PLATFORM_ID:Windows>:comdlg32 ws2_32 winmm>)
target_compile_definitions(vde_sim PRIVATE
  VDE_FONT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/misc/fonts"
  VDE_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/models")

# Configure subsystem and static runtime linking on Windows builds.
if(WIN32)
  set_target_properties(vde_sim PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>)
  if(MSVC)
    target_link_options(vde_sim PRIVATE $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>)
  elseif(MINGW)
    target_link_options(vde_sim PRIVATE -static)
  endif()
endif()

# Copy UI and monospace TrueType fonts next to the compiled executable so runtime lookup succeeds anywhere.
add_custom_command(TARGET vde_sim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:vde_sim>/fonts
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/misc/fonts/Roboto-Medium.ttf
          ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/misc/fonts/Cousine-Regular.ttf
          $<TARGET_FILE_DIR:vde_sim>/fonts
  COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:vde_sim>/models
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/materials.mtl
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/chaser.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/target.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/earth.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/vde_base.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/vde_gantry.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/vde_platform.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/leg_sleeve.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/leg_piston.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/leg_joint.obj
          ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/axis_triad.obj
          $<TARGET_FILE_DIR:vde_sim>/models)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT vde_sim)
