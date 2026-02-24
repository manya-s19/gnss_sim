cmake_minimum_required(VERSION 3.10)
project(gnss_sim)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)

# ── gnss_sim: spoofing simulator (writes CSVs, no OpenGL) ──
add_executable(gnss_sim
    src/main.cpp
    src/Satellite.cpp
    src/Receiver.cpp
    src/Spoofer.cpp
    src/Detector.cpp
)
target_include_directories(gnss_sim PRIVATE
    /opt/homebrew/Cellar/glfw/3.4/include
    /opt/homebrew/include
)

# ── gnss_gl: 3D Earth + satellite OpenGL visualizer ──
add_executable(gnss_gl
    src/main_gl.cpp
    src/Satellite.cpp
    src/Visualizer_gl.cpp
)
target_include_directories(gnss_gl PRIVATE
    /opt/homebrew/Cellar/glfw/3.4/include
    /opt/homebrew/include
)
target_link_libraries(gnss_gl
    ${OPENGL_LIBRARIES}
    /opt/homebrew/Cellar/glfw/3.4/lib/libglfw.dylib
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
)