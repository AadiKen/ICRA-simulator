cmake_minimum_required(VERSION 3.16)
project(StonefishVehicleABridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Freetype REQUIRED)
find_package(Stonefish 1.6.0 EXACT REQUIRED)

add_executable(stonefish_vehicle_a_bridge
    src/main.cpp
    src/VehicleAManager.cpp
)

target_include_directories(stonefish_vehicle_a_bridge PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${FREETYPE_INCLUDE_DIRS}
)

target_link_libraries(stonefish_vehicle_a_bridge PRIVATE
    Stonefish::Stonefish
    ${SDL2_LIBRARIES}
    ${FREETYPE_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

target_compile_definitions(stonefish_vehicle_a_bridge PRIVATE
    BT_EULER_DEFAULT_ZYX
    BT_USE_DOUBLE_PRECISION
)
