# OrbbecSDK
<!-- -->
The Orbbec 3D camera product software development kit，fully supports UVC, realizes driver-free plug-and-play, provides low-level and high-level simple and easy-to-use APIs, and helps developers use it flexibly in different scenarios.


## what is include in the repository

* Doc
> API reference documentation and sample documentation.


* Script
> Windows: timestamp registration script, used to solve the problem of obtaining UVC timestamps and metadata under Windows.

> Linux: udev rules installation scripts to resolve device permission issues.

* Example
> C/C++ samples, including sample project source code and compiled executables.

* SDK
> OrbbecSDK header files and library files, OrbbecSDK running configuration files

## Platform support

| Operating system | Requirement     | Description           |
|------------------|-----------------|-----------------------|
| Windows          | - Windows 10 April 2018 (version 1803, operating system build 17134) release (x64) or higher | The generation of the VS project depends on the installation of the VS version and the cmake version, and supports VS2015/vs2017/vs2019 |
| Linux            | - Linux Ubuntu 16.04/18.04/20.04 (x64)                                                       | Support GCC 7.5                                                                                                                         |
| Arm32            | - Linux Ubuntu 16.04/18.04/20.04                                                             | Support GCC 7.5                                                                                                                         |
| Arm64            | - Linux Ubuntu 18.04/20.04                                                                   | Support GCC 7.5                                                                                                                         |
| MacOS            | - M series chip, 11.0 and above、intel x86 chip, 10.15 and above.                             | supported hardware products: Gemini 2, Gemini 2 L, Astra 2,Gemini 2 XL, Femto Mega ,Gemini 300 series                                                     |

* Note: supported Arm platforms: jestson nano (arm64)、 AGX Orin(arm64)、Orin NX (arm64)、Orin Nano(arm64)、A311D (arm64), Raspberry Pi 4 (arm64), Raspberry Pi 3 (arm32), rk3399 (arm64), other Arm systems, may need to Cross-compile.
* Windows 11, Ubuntu 22.04 and other Linux platforms may also be supported, but have not been fully tested.


## Product support

| **products list** | **firmware version** |
| --- | --- |
| Gemini 335         | 1.2.20                      |
| Gemini 335L         | 1.2.20                     |
| Femto Bolt        | 1.0.6/1.0.9                 |
| Femto Mega        | 1.1.7/1.2.7                 |
| Gemini 2 XL       | Obox: V1.2.5  VL:1.4.54     |
| Astra 2           | 2.8.20                      |
| Gemini 2 L        | 1.4.32                      |
| Gemini 2          | 1.4.60 /1.4.76              |
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

## Getting started

### Environment setup

* Linux:

  Install udev rules file

  ```bash
  cd script
  sudo chmod +x ./install_udev_rules.sh
  sudo ./install_udev_rules.sh
  sudo udevadm control --reload && sudo udevadm trigger
  ```
* Windows:

  Timestamp registration: [follow this: obsensor_metadata_win10](script/obsensor_metadata_win10.md)

* *For more information, please refer to：[Environment Configuration](Doc\tutorial\English/Environment_Configuration.md)*


## Examples

The sample code is located in the `./examples` directory and can be built using CMake.

### Build

```bash
cd OrbbecSDK && mkdir build && cd build && cmake .. && cmake --build . --config Release
```

### Run example

To connect your Orbbec camera to your PC, run the following steps:

```bash
cd OrbbecSDK/build/bin # build output dir
./DepthViewer  # DepthViewer.exe on Windows
```

Notes: On MacOS, sudo privileges are required.

```bash
# MacOS
cd OrbbecSDK/build/bin # build output dir
sudo ./DepthViewer
```

### Use Orbbec SDK in your CMake project

Find and link Orbbec SDK in your CMakeLists.txt file like this:

```cmake
cmake_minimum_required(VERSION 3.1.15)
project(OrbbecSDKTest)

add_executable(${PROJECT_NAME} main.cpp)

# find Orbbec SDK
set(OrbbecSDK_DIR "/your/path/to/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

# link Orbbec SDK
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)
```

## Documents
* API Reference: [doc/api/English/index.html](Doc/api/English/index.html)

## Related links

* [Orbbec Main Page](https://www.orbbec.com/)
* [Orbbec 3D Club](https://3dclub.orbbec3d.com)