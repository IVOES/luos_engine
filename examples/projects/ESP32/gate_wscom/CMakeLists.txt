# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

#ESP_IDE
# list(APPEND EXTRA_COMPONENT_DIRS "../../../../network/robus/HAL/ESP32")
# list(APPEND EXTRA_COMPONENT_DIRS "../../../../tool_services/gate")
# list(APPEND EXTRA_COMPONENT_DIRS "../../../../tool_services/pipe/WS/ESP32_IDF")
# list(APPEND EXTRA_COMPONENT_DIRS "src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
