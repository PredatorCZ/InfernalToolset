cmake_minimum_required(VERSION 3.12)

project(XSIBIN2GLTF VERSION 1.0)

build_target(
  NAME
  xsibin_to_gltf
  TYPE
  ESMODULE
  LINKS
  precore
  infernal_common
  SOURCES
  xsibin_to_gltf.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal XSI BIN to GTLF"
  START_YEAR
  2021)

install(TARGETS xsibin_to_gltf DESTINATION bin)
