cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TalkTalkWifi)

add_compile_options(-std=gnu++11)

list(APPEND EXTRA_COMPONENT_DIRS components)
