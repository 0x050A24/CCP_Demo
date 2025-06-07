cmake_minimum_required(VERSION 3.22)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(Defines_Syms

    $<$<CONFIG:Debug>:DEBUG>
)

# include paths
set(Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F30x/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CCP/Inc
)

# CCP Source paths
set(CCP_Driver_Source
    ${CMAKE_SOURCE_DIR}/Drivers/CCP/Src/ccp.c
    ${CMAKE_SOURCE_DIR}/Drivers/CCP/Src/ccp_interface.c
)

# application sources
set(Application_Src
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gd32f30x_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/systick.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F30x/Source/ARM/startup_gd32f30x_cl.s
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
)

# GD32 LL Drivers
set(GD32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F30x/Source/system_gd32f30x.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_adc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_bkp.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_can.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_crc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_ctc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dac.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dbg.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_dma.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_enet.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_exmc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_exti.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_fmc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_fwdgt.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_gpio.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_i2c.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_misc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_pmu.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_rcu.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_rtc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_sdio.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_spi.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_timer.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_usart.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F30x_standard_peripheral/Source/gd32f30x_wwdgt.c
)

# Drivers Midllewares

# Link directories setup
set(LINK_DIRS
)

# Project static libraries
set(LINK_LIBS
    GD32_Drivers
)

# Interface library for includes and symbols
add_library(GigaFirmware INTERFACE)
target_include_directories(GigaFirmware INTERFACE ${Include_Dirs})
target_compile_definitions(GigaFirmware INTERFACE ${Defines_Syms})

# Create GD32_Drivers static library
add_library(GD32_Drivers OBJECT)
target_sources(GD32_Drivers PRIVATE ${GD32_Drivers_Src})
target_link_libraries(GD32_Drivers PUBLIC GigaFirmware)

# Add application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${Application_Src}
    ${CCP_Driver_Source}
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR(CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
