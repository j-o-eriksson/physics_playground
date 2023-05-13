cmake_minimum_required(VERSION 3.15)
project(Jphys)

add_library(jphys phycpp/particle.cpp)
target_include_directories(jphys PUBLIC phycpp ext/glm)
