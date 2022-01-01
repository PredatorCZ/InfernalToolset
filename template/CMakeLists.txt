cmake_minimum_required(VERSION 3.12)

project(TMP2GLTF VERSION 1.0)

build_target(
  NAME
  template_to_gltf
  TYPE
  ESMODULE
  LINKS
  precore
  infernal_common
  SOURCES
  template_to_gltf.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal TMPDATA to GTLF"
  START_YEAR
  2021)

install(TARGETS template_to_gltf DESTINATION bin)
