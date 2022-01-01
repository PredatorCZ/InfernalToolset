cmake_minimum_required(VERSION 3.12)

project(XBN2GLTF VERSION 1.0)

build_target(
  NAME
  xbn_to_gltf
  TYPE
  ESMODULE
  LINKS
  precore
  infernal_common
  SOURCES
  xbn_to_gltf.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal XBN to GTLF"
  START_YEAR
  2021)

install(TARGETS xbn_to_gltf DESTINATION bin)
