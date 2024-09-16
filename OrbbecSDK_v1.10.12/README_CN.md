# OrbbecSDK
<!-- 本文档面向开发者 -->
新一代 Orbbec 3D 相机产品软件开发套件，全面支持 UVC 协议，实现免驱即插即用，提供低阶和高阶简洁、易用的 API，帮助开发者在不同场景下灵活使用。

## 目录说明

1.Doc
> API 参考文档及示例说明文档。

2.Script
> Windows：时间戳注册脚本，用于解决 Windows 下获取 UVC 时间戳及 Metadata 问题。
> Linux：udev rules 安装脚本，用于解决设备访问权限问题。

3.Example
> C/C++ 示例，包括示例工程源码和已编译好的可执行文件。

4.SDK
> OrbbecSDK 头文件和库文件、OrbbecSDK运行配置文件。

## 操作系統

| Operating system | Requirement    | Description                               |
|------------------|----------------|-------------------------------------------|
| Windows          | - Windows 10 April 2018 (version 1803, operating system build 17134) release (x64) or higher | The generation of the VS project depends on the installation of the VS version and the cmake version, and supports VS2015/vs2017/vs2019 |
| Linux            | - Linux Ubuntu 16.04/18.04/20.04 (x64)                                                       | Support GCC 7.5                                                                                                                         |
| Arm32            | - Linux Ubuntu 16.04/18.04/20.04                                                             | Support GCC 7.5                                                                                                                         |
| Arm64            | - Linux Ubuntu 18.04/20.04                                                                   | Support GCC 7.5                                                                                                                         |
| MacOS            | - M series chip, 11.0 and above、intel x86 chip, 10.15 and above.                             | supported hardware products: Gemini 2, Gemini 2 L, Astra 2,Gemini 2 XL, Femto Mega,Gemini 300 series                                                       |

* 注: 当前版本支持的Arm平台：jestson nano(arm64)、AGX Orin(arm64)、Orin NX (arm64)、Orin Nano(arm64)、A311D(arm64)、树莓派4(arm64)、树莓派3（arm32)、rk3399(arm64), 其它Arm系统，可能需要重新交叉编译。
* Windows 11, Ubuntu 22.04 和其他一些 Linux 平台理论上也支持，但是未经过完整测试”


## 支持产品

| **产品列表**     | **固件版本**                |
|------------------|-----------------------------|
| Gemini 335         | 1.2.20                      |
| Gemini 335L         | 1.2.20                     |
| Femto Bolt       | 1.0.6/1.0.9                 |
| Femto Mega       | 1.1.7/1.2.7                 |
| Gemini 2 XL      | Obox: V1.2.5  VL:1.4.54     |
| Astra 2          | 2.8.20                      |
| Gemini 2 L       | 1.4.32                      |
| Gemini 2         | 1.4.60 /1.4.76              |
| Astra+            | 1.0.22/1.0.21/1.0.20/1.0.19 |
| Femto             | 1.6.7                       |
| Femto W           | 1.1.8                       |
| DaBai             | 2436                        |
| DaBai DCW         | 2460                        |
| DaBai DW          | 2606                        |
| Astra Mini Pro    | 1007                        |
| Gemini E          | 3460                        |
| Gemini E Lite     | 3606                        |
| Gemini            | 3.0.18                      |
| Astra Mini S Pro  | 1.0.05                      |

## 快速开始

### 环境配置

* Linux：

  安装 udev rules 文件

  ```bash
  cd OrbbecSDK/misc/scripts
  sudo chmod +x ./install_udev_rules.sh
  sudo ./install_udev_rules.sh
  sudo udevadm control --reload && sudo udevadm trigger
  ```
* Windows：
  metadat时间戳注册: [obsensor_metadata_win10](misc\scripts\obsensor_metadata_win10.md)
* 有关环境配置的更多信息请参考：[Environment Configuration](doc/tutorial/Chinese/Environment_Configuration.md)


## 示例

示例代码位于./examples目录中，可以使用CMake进行编译

### 编译

    ``bash     cd OrbbecSDK && mkdir build && cd build && cmake .. && cmake --build . --config Release     ``

### 运行示例

    首先连接Orbbec相机，然后运行如下脚本：
    ``bash     
    cd OrbbecSDK/build/bin # build output dir     
    ./DepthViewer  # DepthViewer.exe on Windows     
    ``

注意: 在MacOS, 需要使用sudo权限。

```bash
# MacOS
cd OrbbecSDK/build/bin # build output dir
sudo ./DepthViewer
```


### CMake项目中使用Orbbec SDK

在CMakeLists.txt文件中查找并链接Orbbec SDK：

```cmake
cmake_minimum_required(VERSION 3.1.15)
project(OrbbecSDKTest)

add_executable(${PROJECT_NAME} main.cpp)

# find Orbbec SDK
set(OrbbecSDK_DIR "/your/path/to/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

# link Orbbec SDK
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)


## 文档
* API Reference: [doc/api/English/index.html](Doc/api/English/index.html)

## 相关链接

* [Orbbec Main Page](https://www.orbbec.com/)
* [Orbbec 3D Club](https://3dclub.orbbec3d.com)

