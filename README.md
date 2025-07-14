# GD32F30xProg

## 简介

这是一个基于 GigaDevice GD32F303VCT6 微控制器的嵌入式项目。该项目使用 CMake 进行构建，并已配置好 Visual Studio Code 用于开发、构建和调试。从代码结构来看，这可能是一个磁场定向控制（FOC）电机驱动项目。

## 环境准备

在开始之前，请确保您已安装以下工具。推荐使用 [Winget](https://learn.microsoft.com/en-us/windows/package-manager/winget/) 在 Windows 上快速安装所需的核心构建工具。

打开 PowerShell 或命令提示符，然后运行以下命令：

```powershell
winget install -e --id Arm.GnuArmEmbeddedToolchain
winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT
winget install -e --id Kitware.CMake
winget install -e --id Ninja-build.Ninja
```

除了通过命令行安装的工具，您还需要：

*   **Visual Studio Code**: 以及 `C/C++` 和 `Cortex-Debug` 扩展。
*   **J-Link 驱动**: 用于通过 J-Link 调试器进行硬件调试和烧录 (请从 [SEGGER 官网](https://www.segger.com/downloads/jlink/) 下载并安装)。
*   **Python 3**: 用于运行生成 A2L 文件所需的脚本。

## 如何使用

### 1. 配置项目

首次克隆或打开项目时，需要先配置 CMake。

*   按下 `Ctrl+Shift+P` 打开命令面板。
*   输入并选择 `Tasks: Run Task`。
*   选择 **Configure Project** 任务。
*   这将在 `build` 目录下生成构建所需的文件。

### 2. 编译固件

*   按下 `Ctrl+Shift+B`，然后选择 **Build Firmware**。
*   或者，在命令面板中运行 **Build Firmware** 任务。
*   编译成功后，将在 `build` 目录下生成 `.elf` 和 `.hex` 文件。

### 3. 调试

项目已经配置好使用 J-Link 进行硬件在环调试。

*   确保 J-Link 调试器已连接到目标板和您的电脑。
*   切换到 VS Code 的 "运行和调试" 视图 ( `Ctrl+Shift+D` )。
*   在顶部的下拉菜单中选择 **Debug with JLink**，然后按 `F5` 启动调试。
*   程序将运行到 `main` 函数入口处暂停。

### 4. 烧录固件

您可以通过两种方式将编译好的固件烧录到目标芯片中：

#### 方法一：使用调试器 (例如 J-Link)

这是在开发和调试过程中最便捷的方式。

*   确保 J-Link 等调试器已正确连接。
*   在 VS Code 命令面板 (`Ctrl+Shift+P`) 中运行 **Flash** 任务，即可将固件烧录到目标芯片中。

#### 方法二：使用串口 (UART)

如果手边没有调试器，也可以通过串口进行烧录。

1.  **下载工具**: 您需要从网上下载 `GD32 All-In-One Programmer` 软件。
2.  **进入 ISP 模式**:
    *   将开发板的 `BOOT0` 引脚设置为高电平 (3.3V)。
    *   将 `BOOT1` 引脚设置为低电平 (GND)。
    *   重新给开发板上电或按下复位键，使其进入内置的 Bootloader 模式。
3.  **连接硬件**: 使用 USB-to-Serial 模块将开发板的 `USART0` (通常是 `PA9`/`PA10`) 连接到电脑。
4.  **烧录**:
    *   打开 `GD32 All-In-One Programmer`。
    *   选择正确的芯片型号 (`GD32F303VC`)。
    *   选择对应的串口号。
    *   加载 `build` 目录下生成的 `.hex` 文件 (`GD32F30xProg.hex`)。
    *   点击 "Download" 或 "Program" 开始烧录。
5.  **运行程序**: 烧录完成后，将 `BOOT0` 引脚恢复为低电平，重新上电或复位，新烧录的程序即可运行。

## 可用任务

您可以通过 `Ctrl+Shift+P` -> `Tasks: Run Task` 来执行以下预定义的任务：

*   **一键构建与部署（Build and Deploy）**：自动依次执行项目配置、编译和A2L文件生成。
*   **项目配置（Configure Project）**：运行CMake生成构建文件。
*   **固件编译（Build Firmware）**：编译项目，生成.elf和.hex文件。
*   **生成A2L文件（Generate A2L File）**：从ELF文件生成A2L校准文件。
*   **清理构建产物（Clean Build Artifacts）**：清理build目录下的所有编译产物。
*   **固件烧录（Flash Firmware）**：使用J-Link将固件烧录到硬件。
