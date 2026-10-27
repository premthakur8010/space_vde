cmake_minimum_required(VERSION 3.20)
project(space_docking_system VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_USE_MATH_DEFINES GLM_ENABLE_EXPERIMENTAL)

if (WIN32 AND MINGW)
	add_link_options(-static -static-libgcc -static-libstdc++)
endif()

# Default to Release
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# Dependencies (GLM for vector and matrix math)
include(FetchContent)
FetchContent_Declare(
	glm
	GIT_REPOSITORY https://github.com/g-truc/glm.git
	GIT_TAG        6f14f4792a0cde5d0cf2c910506724d61cb95834
)
FetchContent_MakeAvailable(glm)

# ============================================================
# Core Libraries
# ============================================================

# 1. Utils Module
add_library(space_docking_utils STATIC
	src/utils/Logger.cpp
)
target_include_directories(space_docking_utils PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(space_docking_utils PUBLIC glm Threads::Threads)

# 2. Network Module
add_library(space_docking_network STATIC
	src/network/UdpSocket.cpp
)
target_include_directories(space_docking_network PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(space_docking_network PUBLIC space_docking_utils)
if (WIN32)
	target_link_libraries(space_docking_network PUBLIC ws2_32)
endif()

# 3. Dynamics Engine Module (GSP / FSP 6-DOF Simulation)
add_library(space_docking_dynamics_engine STATIC
	src/dynamics_engine/SpacecraftDynamics.cpp
	src/dynamics_engine/RelativeKinematics.cpp
	src/dynamics_engine/FrameTransformation.cpp
	src/dynamics_engine/SpacecraftPoseComputation.cpp
	src/dynamics_engine/DynamicsEngine.cpp
	src/dynamics_engine/XmlConfigLoader.cpp
)
target_include_directories(space_docking_dynamics_engine PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(space_docking_dynamics_engine PUBLIC
	space_docking_utils
	space_docking_network
	glm
	Threads::Threads
)
if (WIN32)
	target_link_libraries(space_docking_dynamics_engine PUBLIC ws2_32 winmm)
endif()

# ============================================================
# Executables
# ============================================================

# Physics Engine Server Executable
add_executable(physics_engine src/dynamics_engine/dynamics_engine_main.cpp)
target_link_libraries(physics_engine PRIVATE space_docking_dynamics_engine)
if (WIN32 AND MINGW)
	target_link_libraries(physics_engine PRIVATE -static -static-libgcc -static-libstdc++)
endif()

# ============================================================
# Automated Testing Suite
# ============================================================
enable_testing()

add_executable(test_math_utils tests/unit/TestMathUtils.cpp)
target_link_libraries(test_math_utils PRIVATE space_docking_utils)
add_test(NAME MathUtilsTest COMMAND test_math_utils)

add_executable(test_protocol tests/unit/TestProtocol.cpp)
target_link_libraries(test_protocol PRIVATE space_docking_network)
add_test(NAME ProtocolTest COMMAND test_protocol)

add_executable(test_dynamics_engine_base tests/unit/TestDynamicsEngineBase.cpp)
target_link_libraries(test_dynamics_engine_base PRIVATE space_docking_dynamics_engine)
add_test(NAME DynamicsEngineBaseTest COMMAND test_dynamics_engine_base)
