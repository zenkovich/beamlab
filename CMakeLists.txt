cmake_minimum_required(VERSION 3.20)
project(BeamLab C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# BEAMLAB_PORTABLE=ON: no -mcpu=native (for builds sent to other machines), see tools/package_macos.sh
option(BEAMLAB_PORTABLE "Build for distribution (no host-specific CPU tuning)" OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------- GLFW (static)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/glfw EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------- ImGui + perfmon
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
add_library(imgui STATIC
  ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
  third_party/imgui_perfmon/PerfmonWidget.cpp third_party/imgui_perfmon/NanoProfiler.cpp)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends third_party/imgui_perfmon)
target_link_libraries(imgui PUBLIC glfw)

# ---------------------------------------------------------------- App
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.h)
add_executable(beamlab ${APP_SOURCES})
target_include_directories(beamlab PRIVATE src third_party/stb)
target_link_libraries(beamlab PRIVATE imgui glfw)
target_compile_definitions(beamlab PRIVATE GL_SILENCE_DEPRECATION BEAMLAB_ROOT="${CMAKE_SOURCE_DIR}")

if(APPLE)
  target_link_libraries(beamlab PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(beamlab PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-sign-compare)
  # Release: full optimization + debug info (keeps profiling/debugging usable)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -g -fno-math-errno -ffp-contract=fast" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -g" CACHE STRING "" FORCE)
  if(BEAMLAB_PORTABLE)
    # distributable build (any Apple Silicon Mac; x86_64 slice for Intel Macs when building universal)
    if(APPLE)
      target_compile_options(beamlab PRIVATE $<$<CONFIG:Release>:-Xarch_arm64 -mcpu=apple-m1>)
    endif()
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    target_compile_options(beamlab PRIVATE $<$<CONFIG:Release>:-mcpu=native>)
  else()
    target_compile_options(beamlab PRIVATE $<$<CONFIG:Release>:-march=native>)
  endif()
endif()

# ---------------------------------------------------------------- Loader test tools (no GL needed)
# (the frame elements the editor's model builds pull in the physics: its sources as for test_physics)
file(GLOB PHYS_SOURCES CONFIGURE_DEPENDS src/phys/*.cpp)
add_executable(test_ror_parser EXCLUDE_FROM_ALL tools/test_ror_parser.cpp src/vehicle/ror_parser.cpp src/game/editor_model.cpp ${PHYS_SOURCES} src/core/jobs.cpp
  src/core/profiler.cpp src/core/util.cpp src/core/math.cpp third_party/imgui_perfmon/NanoProfiler.cpp)
target_include_directories(test_ror_parser PRIVATE src third_party/imgui_perfmon)
target_compile_definitions(test_ror_parser PRIVATE BEAMLAB_ROOT="${CMAKE_SOURCE_DIR}")
# Physics regression tests (no GL): ./build/test_physics [bench]
add_executable(test_physics EXCLUDE_FROM_ALL tests/test_physics.cpp tests/reference_shell.cpp ${PHYS_SOURCES} src/core/jobs.cpp src/core/profiler.cpp
  src/core/math.cpp src/core/util.cpp third_party/imgui_perfmon/NanoProfiler.cpp)
target_include_directories(test_physics PRIVATE src tests third_party/imgui_perfmon)
if(APPLE AND NOT BEAMLAB_PORTABLE)
  target_compile_options(test_physics PRIVATE $<$<CONFIG:Release>:-mcpu=native>)
endif()
add_executable(test_ogre EXCLUDE_FROM_ALL tools/test_ogre.cpp src/vehicle/ogre_mesh.cpp src/vehicle/ogre_material.cpp src/core/util.cpp src/core/math.cpp)
target_include_directories(test_ogre PRIVATE src)
target_compile_definitions(test_ogre PRIVATE BEAMLAB_ROOT="${CMAKE_SOURCE_DIR}")
