cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 17)
project(InfernalToolset VERSION 1.0)

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

set(TPD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party)

set(PUGI_SHARED_LIBS
    ON
    CACHE BOOL "")
set(PC_SHARED_LIB
    ON
    CACHE BOOL "")

set(GLTF_SHARED_OBJECTS
    ON
    CACHE BOOL "")

add_subdirectory(${TPD_PATH}/precore)
include(targetex)
include(chartype)
add_subdirectory(common)

add_subdirectory(${TPD_PATH}/precore/gltf)
add_subdirectory(${TPD_PATH}/precore/spike ${CMAKE_CURRENT_BINARY_DIR}/spike)
set_target_properties(spike PROPERTIES OUTPUT_NAME infernal_toolset)

add_spike_subdir(zap_extract)
add_spike_subdir(tex_conv)
add_spike_subdir(xbn_conv)
add_spike_subdir(vfs_extract)
add_spike_subdir(xsibin_conv)
# add_spike_subdir(make_cache)
add_spike_subdir(template)
