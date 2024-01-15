cmake_minimum_required(VERSION 3.22)
set(BOARD rpi_pico)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nci-zephyr)

target_include_directories(app PRIVATE src/)
target_sources(app PRIVATE src/main.cpp)
