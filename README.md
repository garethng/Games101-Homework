# Homework for Games101
GAMES101-现代计算机图形学入门-闫令琪
课程链接 [GAMES101](https://www.bilibili.com/video/BV1X7411F744)

## macOS 使用

### 安装 vcpkg

```bash
brew install vcpkg
```

### 安装 Eigen

添加并编辑 vcpkg.json 文件，添加 Eigen 依赖

```json
{
    "name": "transformation",
    "version-string": "1.0.0",
    "dependencies": [
        "eigen3"
    ]
}
```

### 安装依赖

```bash
vcpkg install
```

### 修改 CMakeLists.txt

```cmake

# 设置 CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH 
    "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/arm64-osx"
)

set(CMAKE_TOOLCHAIN_FILE "${ENV{HOME}}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

find_package(Eigen3 CONFIG REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

message(STATUS "EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
add_executable (Transformation main.cpp)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)

```

### 编译

```bash
mkdir build
cd build
cmake ..
make
```

