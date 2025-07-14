cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
get_filename_component(PROJECT_FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_PROJECT_NAME ${PROJECT_FOLDER_NAME})

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    __FPU_PRESENT=1
)

add_subdirectory(cmake/GigaFirmware)

# 手动添加使用的 CMSIS-DSP 文件
add_library(CMSISDSP STATIC
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/CommonTables/arm_common_tables.c
)

target_include_directories(CMSISDSP PUBLIC
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F30x/Include
)


target_sources(${PROJECT_NAME} PRIVATE
    Core/Src/usart.c
    Core/Src/gpio.c
    Core/Src/can.c
    Core/Src/tim.c
    Core/Src/foc.c
    Core/Src/adc.c
    Core/Src/temp_table.c
    Core/Src/dma.c
    Core/Src/position_sensor.c
)

target_link_libraries(${PROJECT_NAME}
    GigaFirmware
    CMSISDSP
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file from ELF"
)
