cmake_minimum_required(VERSION 3.20.0)

set(BOARD leftmk_l1)

set(CONF_FILE "prj.conf")
list(APPEND CONF_FILE "debug.conf")

find_package(Zephyr)
project(zkeyboard_app)

target_sources(zkeyboard PRIVATE src/main.c)
