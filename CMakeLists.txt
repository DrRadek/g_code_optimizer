cmake_minimum_required(VERSION 3.15)
project(g_code_optimizer)

set(PlatformToolsetVersion "143")
if (WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PlatformArchitecture "Win64")
elseif(WIN32)
    set(PlatformArchitecture "Win32")
endif()

if (WIN32)
    # Path to trimesh2 build
    set(TRIMESH_LIB_DIR "trimesh2/lib.${PlatformArchitecture}.vs${PlatformToolsetVersion}")
    set(TRIMESH_INCLUDE_DIR "trimesh2/include")

    # Path to cuda_voxelizer build
    set(CUDAVOXELIZER_LIB_DIR "cuda_voxelizer/build/lib.${PlatformArchitecture}.vs${PlatformToolsetVersion}")
    set(CUDAVOXELIZER_INCLUDE_DIR "cuda_voxelizer/include")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS src/*.h)

# Add the executable target
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Define variables for paths
set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
set(THIRD_PARTY_LIBS_DIR "${THIRD_PARTY_DIR}/libs")
set(THIRD_PARTY_INCLUDE_DIR "${THIRD_PARTY_DIR}/include")

# Add include directory for public headers
# Include Eigen and OpenSTL
target_include_directories(${PROJECT_NAME} PUBLIC ${THIRD_PARTY_INCLUDE_DIR})
# Include Trimesh2
include_directories(${THIRD_PARTY_DIR}/${TRIMESH_INCLUDE_DIR})
# Include cuda_voxelizer
include_directories(${THIRD_PARTY_DIR}/${CUDAVOXELIZER_INCLUDE_DIR})

# Add CUDAToolkit
find_package(CUDAToolkit REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE CUDA::cudart)
target_include_directories(${PROJECT_NAME} PRIVATE ${CUDAToolkit_INCLUDE_DIRS})

# Link static libraries
if (WIN32)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${THIRD_PARTY_DIR}/${CUDAVOXELIZER_LIB_DIR}/cuda_voxelizer.lib
        ${THIRD_PARTY_DIR}/${TRIMESH_LIB_DIR}/trimesh.lib
    )

    # Copy DLLs to output directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${THIRD_PARTY_DIR}/${CUDAVOXELIZER_LIB_DIR}/cuda_voxelizer.dll
            ${THIRD_PARTY_DIR}/${CUDAVOXELIZER_LIB_DIR}/cudart64_12.dll
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
endif()

# Group files in Visual Studio
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "Source Files" FILES ${SOURCES})
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "Header Files" FILES ${HEADERS})