# ==============================================================================
# GD32F30x 项目 CMake 配置文件
# 该文件用于配置基于 GD32F30x 微控制器的嵌入式项目构建
# ==============================================================================

cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 编译器和构建设置
# ==============================================================================

# 设置 C 语言标准为 C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 启用详细的构建输出，便于调试构建过程
set(CMAKE_VERBOSE_MAKEFILE ON)

# 启用编译命令导出，便于 IDE 和代码分析工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ==============================================================================
# 构建类型配置
# ==============================================================================

# 如果未指定构建类型，默认使用 Debug 模式
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# ==============================================================================
# 项目配置
# ==============================================================================

# 自动获取项目名称（使用当前目录名）
get_filename_component(PROJECT_FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(CMAKE_PROJECT_NAME ${PROJECT_FOLDER_NAME})

# 包含 ARM GCC 工具链配置
# 该文件定义了交叉编译所需的编译器和链接器设置
include("cmake/gcc-arm-none-eabi.cmake")

# 声明项目
project(${CMAKE_PROJECT_NAME})
message("-- 项目名称: ${CMAKE_PROJECT_NAME}")
message("-- 构建类型: ${CMAKE_BUILD_TYPE}")

# 启用 C 和汇编语言支持
enable_language(C ASM)

# ==============================================================================
# 主可执行目标配置
# ==============================================================================

# 创建主可执行文件目标
add_executable(${PROJECT_NAME})

# 为主目标添加预处理器定义
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ARM_MATH_CM4    # 启用 ARM Cortex-M4 的数学库优化
    __FPU_PRESENT=1 # 声明存在浮点运算单元 (FPU)
)

# ==============================================================================
# 子模块和依赖库配置
# ==============================================================================

# 添加 GigaFirmware 子项目
# 该子项目包含了固件的核心功能模块
add_subdirectory(cmake/GigaFirmware)

# ==============================================================================
# CMSIS-DSP 数学库配置
# ==============================================================================

# 创建 CMSIS-DSP 静态库
# 仅包含项目中实际使用的数学函数，减少编译时间和固件大小
add_library(CMSISDSP STATIC
    # 快速数学函数
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c
    # 公共数据表（包含三角函数查找表等）
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Source/CommonTables/arm_common_tables.c
)

# 为 CMSIS-DSP 库配置包含目录
target_include_directories(CMSISDSP PUBLIC
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/CMSIS-DSP/Include  # CMSIS-DSP 头文件
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include       # CMSIS 核心头文件
)

# ==============================================================================
# 主目标的包含目录配置
# ==============================================================================

# 为主可执行文件配置头文件搜索路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include      # CMSIS 核心接口
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F30x/Include # GD32F30x 特定头文件
)

# ==============================================================================
# 主目标的源文件配置
# ==============================================================================

# 添加应用程序核心源文件
# 这些是项目的主要功能模块
target_sources(${PROJECT_NAME} PRIVATE
    # 外设驱动模块
    Core/Src/usart.c         # UART 串口通信
    Core/Src/gpio.c          # GPIO 引脚控制
    Core/Src/can.c           # CAN 总线通信
    Core/Src/tim.c           # 定时器配置与控制
    Core/Src/adc.c           # ADC 模数转换
    Core/Src/dma.c           # DMA 直接内存访问
    
    # 应用功能模块
    Core/Src/foc.c           # FOC 电机控制算法
    Core/Src/position_sensor.c # 位置传感器处理
    Core/Src/temp_table.c    # 温度查找表
)

# ==============================================================================
# 库链接配置
# ==============================================================================

# 链接必要的库到主可执行文件
target_link_libraries(${PROJECT_NAME}
    GigaFirmware    # 自定义固件库
    CMSISDSP        # CMSIS 数学函数库
)

# ==============================================================================
# 构建后处理
# ==============================================================================

# 构建完成后自动生成 HEX 文件
# HEX 文件用于烧录到微控制器
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "正在从 ELF 文件生成 HEX 烧录文件..."
    VERBATIM
)
