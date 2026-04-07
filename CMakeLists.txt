cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bakalarka_can_master)
spiffs_create_partition_image(spiffs spiffs FLASH_IN_PROJECT)
