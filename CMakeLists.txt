cmake_minimum_required(VERSION 3.16)

project(Realistic-RayTracing LANGUAGES CXX)

# Un build CMake dans le dossier source écraserait le Makefile du projet.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "Build CMake dans le dossier source interdit (il écraserait le Makefile).\n"
        "Utiliser un dossier de build séparé, par ex. : cmake -S . -B build")
endif()

# Release par défaut : le raytracing est très lent sans optimisation.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Type de build" FORCE)
endif()

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads REQUIRED)

set(SOURCES
    main.cpp
    src/Camera.cpp
    src/Trackball.cpp
    src/imageLoader.cpp
    src/Mesh.cpp
    src/BVH.cpp
)

# Listés uniquement pour qu'ils apparaissent dans l'arborescence de Qt Creator.
set(HEADERS
    src/AABB.h
    src/BVH.h
    src/Camera.h
    src/Line.h
    src/Material.h
    src/matrixUtilities.h
    src/Mesh.h
    src/Plane.h
    src/Ray.h
    src/Scene.h
    src/Sphere.h
    src/Square.h
    src/Trackball.h
    src/Triangle.h
    src/Vec3.h
    src/imageLoader.h
)

add_executable(main ${SOURCES} ${HEADERS})

target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR})
target_compile_options(main PRIVATE -Wall)
target_link_libraries(main PRIVATE GLUT::GLUT OpenGL::GLU OpenGL::GL Threads::Threads m)

# Le programme utilise des chemins relatifs (data/, img/, ./rendu.ppm) : on place
# l'exécutable à la racine du projet. Qt Creator le lance depuis le dossier de
# l'exécutable, donc depuis la racine.
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
