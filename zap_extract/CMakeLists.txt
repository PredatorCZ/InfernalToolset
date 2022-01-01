cmake_minimum_required(VERSION 3.12)

project(ZAPExtract VERSION 1.0)

build_target(
  NAME
  zap_extract
  TYPE
  ESMODULE
  LINKS
  precore
  infernal_common
  SOURCES
  zap_extract.cpp
  AUTHOR
  "Lukas Cone"
  DESCR
  "Infernal ZAP extractor"
  START_YEAR
  2021)

install(TARGETS zap_extract DESTINATION bin)
