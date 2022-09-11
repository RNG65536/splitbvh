cmake_minimum_required(VERSION 3.12)
project(splitbvh)

include(D:/deps.cmake)
include(assign_source_group.cmake)

set(CMAKE_CXX_STANDARD 17)

find_package(openmp REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(glm REQUIRED)

# main
file(GLOB_RECURSE main_srcs "src/*.*")
assign_source_group(${main_srcs})

add_executable(${PROJECT_NAME}
${main_srcs}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX)

target_include_directories(${PROJECT_NAME} PRIVATE
    src/bvh
    src/common
    src/geometry
    src/loader
    src/bvh_ref
    src
)

target_link_libraries(${PROJECT_NAME}
    glm
)