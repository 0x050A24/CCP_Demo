# ==============================================================================
# GD32F30x 项目 CMake 配置文件
# 该文件用于配置基于 GD32F30x 微控制器的嵌入式项目构建
# ==============================================================================

cmake_minimum_required ( VERSION 3.22 )

# ==============================================================================
# 编译器和构建设置
# ==============================================================================

# 设置 C 语言标准为 C11
set ( CMAKE_C_STANDARD 11 )
set ( CMAKE_C_STANDARD_REQUIRED ON )
set ( CMAKE_C_EXTENSIONS ON )

# 启用详细的构建输出，便于调试构建过程
set ( CMAKE_VERBOSE_MAKEFILE ON )

# 启用编译命令导出，便于 IDE 和代码分析工具使用
set ( CMAKE_EXPORT_COMPILE_COMMANDS TRUE )

# ==============================================================================
# 构建类型配置
# ==============================================================================

# 如果未指定构建类型，默认使用 Debug 模式
if ( NOT CMAKE_BUILD_TYPE )
    set ( CMAKE_BUILD_TYPE "Debug" )
endif ()

# ==============================================================================
# 项目配置
# ==============================================================================

# 自动获取项目名称（使用当前目录名）
get_filename_component ( PROJECT_FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
set ( CMAKE_PROJECT_NAME ${PROJECT_FOLDER_NAME} )

# 包含 ARM GCC 工具链配置
# 该文件定义了交叉编译所需的编译器和链接器设置
include ( "cmake/gcc-arm-none-eabi.cmake" )

# 声明项目
project ( ${CMAKE_PROJECT_NAME} )

# 启用 C 和汇编语言支持
enable_language ( C ASM )

# ==============================================================================
# 主可执行目标配置
# ==============================================================================

# 创建主可执行文件目标
add_executable ( ${PROJECT_NAME} 
    ${CMAKE_SOURCE_DIR}/Application/Src/main.c
    ${CMAKE_SOURCE_DIR}/Application/Src/gd32f30x_it.c
    ${CMAKE_SOURCE_DIR}/Application/Src/foc.c
)

# 为主目标添加预处理器定义
target_compile_definitions ( ${PROJECT_NAME} PRIVATE
    ARM_MATH_CM4 # 启用 ARM Cortex-M4 的数学库优化
    __FPU_PRESENT=1 # 声明存在浮点运算单元 (FPU)
)

# ==============================================================================
# 子模块和依赖库配置
# ==============================================================================
add_subdirectory ( Drivers )
add_subdirectory ( Core )
add_subdirectory ( Communication )
add_subdirectory ( Middleware )
add_subdirectory ( Application )

# ==============================================================================
# CMSIS-DSP 数学库配置
# ==============================================================================

# 创建 CMSIS-DSP 静态库
# 仅包含项目中实际使用的数学函数，减少编译时间和固件大小



# ==============================================================================
# 库链接配置
# ==============================================================================

# 链接必要的库到主可执行文件
target_link_libraries ( ${PROJECT_NAME}
    PRIVATE
        Application
        Drivers
        Communication
        Middleware
        Core
)

# ==============================================================================
# 构建后处理
# ==============================================================================

# 构建完成后自动生成 HEX 文件
# HEX 文件用于烧录到微控制器
add_custom_command ( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    VERBATIM
)

target_link_options ( ${PROJECT_NAME} PRIVATE
    -Wl,--no-warn-rwx-segment
)
