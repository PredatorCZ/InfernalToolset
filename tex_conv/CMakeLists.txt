cmake_minimum_required(VERSION 3.12)

project(TEX2DDS VERSION 1.0)

build_target(
  NAME
  tex_to_dds
  TYPE
  ESMODULE
  LINKS
  precore
  SOURCES
  tex_to_dds.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal texture to dds"
  START_YEAR
  2021)

install(TARGETS tex_to_dds DESTINATION bin)
