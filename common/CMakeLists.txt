cmake_minimum_required(VERSION 3.12)

project(InfernalCommon VERSION 1.0)

file(GLOB ZLIB_SOURCES "${TPD_PATH}/zlib/*.c")

build_target(
  NAME
  infernal_common
  TYPE
  SHARED
  LINKS
  precore
  gltf-shared
  INCLUDES
  ${TPD_PATH}/zlib
  SOURCES
  src/gltf.cpp
  src/zlib.cpp
  ${ZLIB_SOURCES}
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal Toolset shared code"
  START_YEAR
  2021)

target_include_directories(infernal_common PUBLIC include)
target_link_libraries(infernal_common -Wl,-z,defs)
target_compile_definitions(
  infernal_common
  PRIVATE IT_EXPORT
  INTERFACE IT_IMPORT)

install(TARGETS infernal_common DESTINATION bin)
