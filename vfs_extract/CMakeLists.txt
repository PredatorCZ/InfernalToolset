cmake_minimum_required(VERSION 3.12)

project(VFSExtract VERSION 1.0)

build_target(
  NAME
  vfs_extract
  TYPE
  ESMODULE
  LINKS
  precore
  infernal_common
  SOURCES
  vfs_extract.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal VFS extractor"
  START_YEAR
  2021)

install(TARGETS vfs_extract DESTINATION bin)
