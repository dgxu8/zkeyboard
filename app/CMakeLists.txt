cmake_minimum_required(VERSION 3.20.0)

set(BOARD leftmk_l1)

set(CONF_FILE "prj.conf")
list(APPEND CONF_FILE "debug.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app LANGUAGES C VERSION 1.0.0)

zephyr_include_directories(inc)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
