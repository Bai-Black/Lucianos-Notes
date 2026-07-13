# C++ 项目的环境管理

C++ 没有一个完全对应 Python `uv` 的统一工具。

一个 Python 项目的环境，通常可以概括为：

> Python 解释器版本 + Python 依赖版本

而一个 C++ 项目的构建环境至少包含：

> 操作系统 + CPU 架构 + 编译器工具链 + C++ 标准 + 构建类型 + 第三方库及其构建选项

例如，同一个第三方库使用 MSVC 和 GCC 编译，或者分别为 x64 和 ARM64 编译，得到的二进制文件通常不能混用。即使平台和编译器相同，Debug、Release、静态链接、动态链接等选项也可能影响二进制兼容性。

因此，C++ 的环境管理不是由单一工具完成的，而是由以下三部分共同组成：

1. **编译工具链**：把源代码转换为特定平台的二进制文件。
2. **构建系统与构建器**：组织编译目标，并调用工具链完成构建。
3. **包管理器**：获取、构建和管理第三方依赖。

本文使用以下组合建立一套项目级工作流：

> MSVC / GCC / Clang + CMake + Ninja + vcpkg Manifest + CMake Presets

其中 Windows 示例以 **MSVC** 为主。GCC 或 Clang 可以替换编译工具链，但 CMake、vcpkg 和整体项目结构仍然可以保持不变。

---

---

---

# 编译工具链

编译工具链负责把 C++ 源代码转换为操作系统能够加载的程序或库。

## 从源代码到可执行文件

一个 C++ 程序通常经历以下阶段：

```text
源文件
  ↓ 预处理
翻译单元
  ↓ 编译
汇编代码
  ↓ 汇编
目标文件
  ↓ 链接
可执行文件或库
```

### 预处理

预处理器负责处理以 `#` 开头的指令，例如：

```cpp
#include <iostream>
#define VALUE 42
```

`#include` 并不是在运行时加载文件，而是在编译前把头文件内容引入当前翻译单元。

### 编译与汇编

编译器检查 C++ 语法和类型，把翻译单元转换为目标平台的指令，随后生成目标文件。

Windows 上的目标文件通常使用 `.obj` 扩展名，Linux 等平台通常使用 `.o` 扩展名。

### 链接

一个项目往往包含多个目标文件和库。链接器负责解析它们之间的符号引用，并生成：

- 可执行文件；
- 静态库；
- 动态库。

例如，源文件可以只看到函数的声明，而函数的实现位于另一个目标文件或库中。链接器负责在构建的最后阶段找到该实现。

---

## 编译器驱动程序

平时直接运行的 `cl`、`g++`、`clang++` 通常是编译器驱动程序。它根据参数依次调用预处理器、编译器、汇编器和链接器，因此一个简单程序可以用一条命令完成构建。

MSVC：

```powershell
cl /std:c++20 /EHsc main.cpp
```

GCC：

```shell
g++ -std=c++20 main.cpp -o app
```

Clang：

```shell
clang++ -std=c++20 main.cpp -o app
```

直接调用编译器适合理解原理和构建单文件程序。当项目包含多个源文件、第三方库、测试和多种构建配置时，手动维护编译命令会迅速变得困难，因此需要构建系统。

---

## 常见工具链

### MSVC

MSVC 是 Microsoft 提供的 C/C++ 工具链，主要用于 Windows 开发。

常见组件包括：

- `cl.exe`：编译器驱动程序；
- `link.exe`：链接器；
- Microsoft C++ 标准库；
- MSVC 运行库；
- Visual Studio Debugger；
- 与 Windows SDK 配合使用的头文件和库。

MSVC 可以通过 Visual Studio 或 Visual Studio Build Tools 安装。安装时还应选择对应的 C++ 开发工作负载和 Windows SDK。

MSVC 的工具较多，需要设置 `Path`、`INCLUDE`、`LIB` 等一组相互匹配的环境变量。因此，通常应从以下终端开始工作：

- Developer Command Prompt for Visual Studio；
- Developer PowerShell for Visual Studio。

这些终端会为当前会话临时配置工具链环境。相比手动把大量 MSVC 目录永久加入系统环境变量，这种方式更可靠，也更容易在多个工具链版本之间切换。

### GCC

GCC 是 GNU Compiler Collection。其 C++ 编译器驱动程序为 `g++`，在 Linux 环境中非常常见，也可以通过 MinGW-w64 等方式用于 Windows。

### Clang

Clang 是 LLVM 项目中的编译器前端，C++ 编译器驱动程序为 `clang++`。

Clang 可以根据平台配合不同的标准库和链接器。例如，在 Windows 上可以与 Microsoft 的标准库和 SDK 配合，在 Linux 上可以使用 libstdc++ 或 libc++。

因此，**编译器、标准库、运行库和链接器是相互关联但并不完全等同的组件**。

---

## C++ 标准与编译器版本

C++20、C++23 表示语言标准，不表示编译器版本。

同一个 C++ 标准需要由不同编译器分别实现，而且不同编译器版本对标准特性的支持程度可能不同。因此，项目既需要声明使用的 C++ 标准，也需要约定能够支持该标准的编译器版本。

在 CMake 中，应当为具体目标声明所需标准：

```cmake
target_compile_features(app PRIVATE cxx_std_20)
```

不要完全依赖 IDE 或编译器的默认语言标准。

---

## 构建配置

### Debug

Debug 构建主要用于开发和调试，通常具有以下特点：

- 保留调试信息；
- 优化程度较低；
- 更容易单步执行和观察变量；
- 可能启用额外的运行时检查。

### Release

Release 构建主要用于性能测试和发布，通常具有以下特点：

- 开启编译优化；
- 调试体验相对较差；
- 最终程序的性能和体积更接近发布状态。

Debug 与 Release 的目标文件和库不应随意混用。项目应为不同配置使用独立的构建目录。

---

## 二进制兼容性

C++ 第三方库的二进制兼容性可能受到以下因素影响：

- 操作系统；
- CPU 架构；
- 编译器及其版本；
- 标准库和运行库；
- Debug 或 Release；
- 静态链接或动态链接；
- 编译选项；
- 库自身的可选功能。

因此，不能只根据“库名和版本相同”就断定两个二进制库能够混用。这也是 C++ 包管理器需要掌握工具链和构建选项的原因。

---

---

---

# 构建系统与构建器

当项目只有一个源文件时，可以直接调用编译器。随着项目扩大，还需要解决以下问题：

- 管理大量源文件和头文件；
- 只重新编译发生变化的部分；
- 描述可执行程序与库之间的关系；
- 管理 Debug、Release 等构建配置；
- 在不同操作系统和编译器之间复用项目配置；
- 查找和链接第三方依赖；
- 运行测试、安装和打包。

这些职责由构建系统承担。

---

## CMake 与底层构建器

CMake 经常被称为构建工具，但更准确地说，它是一个**构建系统生成器**。

CMake 读取 `CMakeLists.txt`，检测当前工具链和依赖，然后生成另一种构建系统所需的规则。真正根据规则执行增量构建的工具通常是 Ninja、Make 或 MSBuild。

```text
CMakeLists.txt
      ↓ CMake Configure / Generate
Ninja、Make 或 Visual Studio 构建规则
      ↓ Build
编译器和链接器
      ↓
可执行文件或库
```

常见组合包括：

| CMake Generator | 底层构建器 | 常见平台 |
|---|---|---|
| Ninja | Ninja | Windows、Linux、macOS |
| Unix Makefiles | Make | Linux、macOS |
| Visual Studio | MSBuild | Windows |
| Ninja Multi-Config | Ninja | Windows、Linux、macOS |

编辑器或 IDE 位于这些工具的更上层。Visual Studio、VS Code、CLion 可以调用 CMake 和底层构建器，但项目的唯一构建定义不应只存在于某个 IDE 的个人设置中。

---

## CMake 的工作阶段

### 1. Configure

CMake 读取项目文件并检测环境，包括：

- 操作系统和 CPU 架构；
- C/C++ 编译器；
- 编译器功能；
- 项目选项；
- 第三方依赖。

配置结果会被保存在构建目录的 CMake Cache 中。

### 2. Generate

CMake 根据配置结果生成 Ninja、Make 或 MSBuild 能够理解的构建规则。

Configure 和 Generate 通常由同一次 `cmake` 命令完成：

```powershell
cmake -S . -B build -G Ninja
```

### 3. Build

构建阶段调用底层构建器，再由底层构建器调用编译器和链接器：

```powershell
cmake --build build
```

使用 `cmake --build` 而不是直接调用 `ninja` 或 `msbuild`，可以让操作命令保持跨平台一致。

### 4. Test

CTest 负责运行由 CMake 项目注册的测试：

```powershell
ctest --test-dir build --output-on-failure
```

### 5. Install

CMake Install 将构建产物、头文件等整理到指定的安装目录：

```powershell
cmake --install build
```

安装不等于编译。它是构建完成后对产物进行整理和部署的阶段。

---

## 相关文件

### `CMakeLists.txt`

`CMakeLists.txt` 是 CMake 项目的构建定义文件，可以描述：

- 项目名称和版本；
- 使用的编程语言；
- 可执行程序和库；
- 源文件；
- C++ 标准；
- 编译选项；
- 第三方依赖；
- 测试和安装规则。

一个最小项目如下：

```cmake
cmake_minimum_required(VERSION 3.25)

project(cpp_demo VERSION 0.1.0 LANGUAGES CXX)

add_executable(cpp_demo
    src/main.cpp
)

target_compile_features(cpp_demo PRIVATE cxx_std_20)
```

### 子目录中的 `CMakeLists.txt`

大型项目可以在子目录内建立新的 `CMakeLists.txt`，再由根目录添加：

```cmake
add_subdirectory(src)
add_subdirectory(tests)
```

根目录负责项目级配置，子目录负责自身目标，可以避免一个文件无限增长。

### `CMakePresets.json`

`CMakePresets.json` 保存项目成员共享的配置、构建和测试预设。

它可以统一：

- Generator；
- 构建目录；
- Debug 或 Release；
- Toolchain 文件；
- CMake Cache 变量；
- 构建和测试命令。

该文件通常应提交 Git。

### `CMakeUserPresets.json`

`CMakeUserPresets.json` 用于个人机器上的本地配置，例如个人安装的 SDK 路径或实验性编译器。

它不应成为团队构建项目的必要条件，通常应加入 `.gitignore`。

### `CMakeCache.txt`

CMake Cache 位于构建目录，保存工具链检测结果和配置变量。

它由 CMake 管理，不应提交 Git，也不应作为项目配置文件手动维护。当工具链、Generator 或关键环境发生较大变化时，删除对应构建目录并重新配置通常比直接修改 Cache 更可靠。

### `build/`

构建目录包含：

- CMake Cache；
- 生成的构建规则；
- 中间目标文件；
- 可执行文件和库；
- 调试信息；
- 当前构建所需的依赖安装树。

构建目录不是项目源文件的一部分，通常应加入 `.gitignore`。

---

## Out-of-source Build

推荐把所有构建产物放入独立目录：

```powershell
cmake -S . -B build
cmake --build build
```

而不是让生成文件与源代码混在一起。

不同配置也应使用不同目录：

```text
build/debug/
build/release/
```

这样可以避免 CMake Cache、目标文件和依赖配置互相污染。

---

## Target-based CMake

现代 CMake 以 Target 为中心。

Target 可以是：

- 可执行程序；
- 静态库；
- 动态库；
- 接口库；
- 从第三方包导入的库。

例如：

```cmake
add_library(math src/math.cpp)

target_include_directories(math
    PUBLIC
        include
)

target_compile_features(math PUBLIC cxx_std_20)

add_executable(app src/main.cpp)
target_link_libraries(app PRIVATE math)
```

`target_link_libraries(app PRIVATE math)` 不仅表示链接 `math`，还允许 CMake 把 `math` 的必要使用要求传播给 `app`。

### `PRIVATE`、`PUBLIC` 与 `INTERFACE`

- `PRIVATE`：只有当前 Target 自己需要；
- `PUBLIC`：当前 Target 和使用它的 Target 都需要；
- `INTERFACE`：当前 Target 自己不需要，使用它的 Target 需要。

这些关键字可以用于头文件目录、编译宏、编译特性和链接依赖。

相比之下，全局使用 `include_directories()`、`link_directories()` 或修改所有目标的编译参数，会让依赖范围变得模糊，不适合作为现代项目的主要工作方式。

---

## 单配置与多配置 Generator

Ninja 和 Unix Makefiles 通常是单配置 Generator。Debug 或 Release 在配置阶段确定：

```powershell
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
```

Visual Studio 和 Ninja Multi-Config 通常是多配置 Generator。构建时再选择配置：

```powershell
cmake --build build --config Release
```

为了屏蔽这些命令差异，项目可以使用 CMake Presets 为每种受支持的配置提供统一入口。

---

---

---

# 包管理器

C++ 第三方依赖可能以不同形式存在：

- 只有头文件的库；
- 需要从源码构建的库；
- 预编译的静态库或动态库；
- 操作系统提供的系统库；
- 带有多个可选功能的复杂框架。

手动下载、编译和配置每个库会带来以下问题：

- 难以记录准确版本；
- 难以处理间接依赖；
- 不同开发者的安装目录不同；
- Debug、Release、架构和链接方式容易不匹配；
- 新机器难以恢复项目环境。

包管理器负责声明、获取、构建和缓存这些依赖。

---

## vcpkg

vcpkg 是一个跨平台 C/C++ 包管理器。

它支持两种主要使用方式：

### Classic 模式

在命令行中直接安装包：

```powershell
vcpkg install fmt
```

这种方式适合临时实验，但依赖声明主要存在于当前 vcpkg 实例，不适合作为项目级标准工作流。

### Manifest 模式

在项目根目录使用 `vcpkg.json` 声明依赖：

```json
{
  "name": "cpp-demo",
  "version-string": "0.1.0",
  "dependencies": [
    "fmt"
  ]
}
```

Manifest 文件可以提交 Git，使依赖声明跟随项目。本文使用 Manifest 模式。

---

## `vcpkg.json`

### 项目信息

```json
{
  "name": "cpp-demo",
  "version-string": "0.1.0"
}
```

vcpkg 包名通常使用小写字母、数字和连字符。

### 依赖列表

```json
{
  "dependencies": [
    "fmt",
    "nlohmann-json"
  ]
}
```

项目只需要声明直接依赖，间接依赖由 vcpkg 根据各个包的配方解析。

### Features

部分包提供可选功能：

```json
{
  "dependencies": [
    {
      "name": "opencv",
      "features": ["jpeg", "png"]
    }
  ]
}
```

Features 会影响依赖图和生成的二进制包。

### `builtin-baseline`

`builtin-baseline` 指向 vcpkg 内置注册表的一个 Git 提交：

```json
{
  "name": "cpp-demo",
  "version-string": "0.1.0",
  "dependencies": ["fmt"],
  "builtin-baseline": "<40 位 Git 提交哈希>"
}
```

它为注册表中的所有包确定一个默认版本集合。可以使用以下命令添加初始 baseline：

```powershell
vcpkg x-update-baseline --add-initial-baseline
```

更新整个版本集合：

```powershell
vcpkg x-update-baseline
```

更新 baseline 可能改变多个直接或间接依赖的版本，应在更新后重新构建并运行测试。

### 版本约束

最低版本约束使用 `version>=`：

```json
{
  "dependencies": [
    {
      "name": "fmt",
      "version>=": "11.0.0"
    }
  ]
}
```

`version>=` 表示最低要求，不表示精确锁定。

若确实需要强制使用某个具体版本，可以使用 `overrides`。Overrides 会忽略其他位置对该包提出的版本约束，因此应谨慎使用。

---

## Triplet

Triplet 描述 vcpkg 构建库时使用的目标配置，例如：

```text
x64-windows
x64-windows-static
x64-linux
arm64-windows
```

Triplet 通常包含：

- 目标 CPU 架构；
- 目标平台；
- 动态或静态链接策略；
- 其他平台相关构建设置。

Triplet 不是完整的编译环境锁定文件，但它是区分不同二进制配置的重要组成部分。

---

## Binary Cache

从源码构建大型 C++ 依赖可能耗时较长。vcpkg 可以缓存已经构建的二进制包。

当包版本、Triplet、编译设置等输入没有变化时，新项目或新构建目录可以复用缓存，而不必重新编译依赖。

Binary Cache 是性能优化，不是应提交到 Git 的项目源文件。

---

## vcpkg 与 CMake 的分工

vcpkg 和 CMake 解决不同问题：

```text
vcpkg.json
    ↓
vcpkg 获取并构建依赖
    ↓
CMake find_package() 找到依赖
    ↓
target_link_libraries() 将依赖连接到目标
```

例如，在 `vcpkg.json` 中声明 fmt：

```json
{
  "dependencies": ["fmt"]
}
```

再在 `CMakeLists.txt` 中使用 fmt：

```cmake
find_package(fmt CONFIG REQUIRED)
target_link_libraries(cpp_demo PRIVATE fmt::fmt)
```

三者的职责分别是：

- `vcpkg.json`：声明并准备依赖；
- `find_package()`：查找依赖提供的 CMake Package；
- `target_link_libraries()`：建立当前 Target 与依赖 Target 的关系。

`find_package()` 本身通常不负责从网络下载依赖。

---

## CMake Toolchain 文件

vcpkg 通过以下 Toolchain 文件接入 CMake：

```text
<vcpkg-root>/scripts/buildsystems/vcpkg.cmake
```

手动配置时可以使用：

```powershell
cmake -S . -B build `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
```

Toolchain 文件会在 CMake 配置的早期阶段执行，因此应在第一次配置构建目录时指定。若一个构建目录最初没有使用 vcpkg Toolchain，后来才添加，通常应删除该构建目录并重新配置。

实际项目中可以把 Toolchain 文件放入 `CMakePresets.json`，避免每次手动输入。

---

## vcpkg 的复现边界

提交带有 baseline 的 `vcpkg.json`，可以固定依赖注册表的版本集合，但它不等于整个开发环境的完整快照。

它没有自动固定：

- 操作系统版本；
- MSVC、GCC 或 Clang 的版本；
- Windows SDK；
- CMake 和 Ninja 版本；
- 所有环境变量；
- 项目外部的系统依赖。

因此，更准确的描述是：

> vcpkg Manifest 管理项目的依赖声明和版本选择；工具链与构建参数还需要由文档、Presets、CI 或容器进一步约束。

---

## Conan

Conan 是另一种跨平台 C/C++ 包管理器。它不仅能消费第三方库，还适合制作、发布和管理自己的二进制包。

相较于本文的 vcpkg 主线，Conan 更适合以下场景：

- 需要发布组织内部的 C++ 库；
- 使用私有制品仓库；
- 需要复杂的交叉编译；
- 同一个包需要维护多种编译器、架构和构建选项；
- 需要 Profiles、包修订和 Lockfiles 等更完整的二进制包管理机制。

对于一般应用项目和开源依赖消费，可以先使用 vcpkg 建立工作流。需要管理和发布自有 C++ 包时，再进一步学习 Conan。

---

---

---

# 标准工作流

本节使用以下组合建立一个可运行的示例项目：

> Windows + x64 + MSVC + C++20 + CMake + Ninja + vcpkg

在 Linux 或 macOS 上，可以替换工具链和安装方式，项目文件与主要命令保持基本一致。

---

## 1. 安装基础开发工具

Windows 需要：

- Git；
- Visual Studio 或 Visual Studio Build Tools；
- MSVC C++ 工具链；
- Windows SDK；
- CMake；
- Ninja；
- vcpkg。

安装 Visual Studio 时，可以选择 **Desktop development with C++** 工作负载。

CMake 和 Ninja 可以由 Visual Studio 工作负载提供，也可以独立安装。无论使用哪种方式，都应先确认当前终端能够找到它们：

```powershell
cl
cmake --version
ninja --version
git --version
```

运行 `cl` 时即使提示没有输入文件，只要能够显示编译器版本，通常就说明开发者终端已经正确加载 MSVC 环境。

---

## 2. 安装 vcpkg

选择一个固定目录克隆 vcpkg：

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\dev\vcpkg
cd C:\dev\vcpkg
.\bootstrap-vcpkg.bat
```

在当前 PowerShell 会话中设置：

```powershell
$env:VCPKG_ROOT = "C:\dev\vcpkg"
$env:Path = "$env:VCPKG_ROOT;$env:Path"
```

检查：

```powershell
vcpkg version
```

`VCPKG_ROOT` 的作用是让项目配置能够找到 vcpkg 的根目录；把根目录加入当前会话的 `Path` 后，可以直接输入 `vcpkg` 运行程序。

如果选择永久设置环境变量，应确保它始终指向当前实际使用的 vcpkg 实例。团队项目还可以进一步固定 vcpkg 工具本身的版本。

---

## 3. 建立项目目录

```powershell
mkdir cpp-demo
cd cpp-demo
git init
mkdir src
mkdir tests
```

项目最终结构如下：

```text
cpp-demo/
├── .gitignore
├── CMakeLists.txt
├── CMakePresets.json
├── vcpkg.json
├── src/
│   └── main.cpp
└── tests/
    └── basic_test.cpp
```

---

## 4. 编写程序

`src/main.cpp`：

```cpp
#include <fmt/core.h>

int main()
{
    fmt::println("Hello, C++ environment!");
    return 0;
}
```

`tests/basic_test.cpp`：

```cpp
int main()
{
    const int result = 20 + 22;
    return result == 42 ? 0 : 1;
}
```

CTest 根据程序退出码判断测试结果：返回 `0` 表示成功，非 `0` 表示失败。

---

## 5. 创建 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.25)

project(cpp_demo VERSION 0.1.0 LANGUAGES CXX)

add_executable(cpp_demo
    src/main.cpp
)

target_compile_features(cpp_demo PRIVATE cxx_std_20)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(cpp_demo PRIVATE fmt::fmt)

include(CTest)

if(BUILD_TESTING)
    add_executable(basic_test
        tests/basic_test.cpp
    )

    target_compile_features(basic_test PRIVATE cxx_std_20)
    add_test(NAME basic_test COMMAND basic_test)
endif()

install(TARGETS cpp_demo
    RUNTIME DESTINATION bin
)
```

这里定义了两个 Target：

- `cpp_demo`：主程序；
- `basic_test`：测试程序。

`include(CTest)` 默认提供 `BUILD_TESTING` 选项，并启用 CTest 支持。

---

## 6. 创建 `vcpkg.json`

先创建依赖清单：

```json
{
  "name": "cpp-demo",
  "version-string": "0.1.0",
  "dependencies": [
    "fmt"
  ]
}
```

然后添加初始 baseline：

```powershell
vcpkg x-update-baseline --add-initial-baseline
```

该命令会把实际的 `builtin-baseline` 写入 `vcpkg.json`。生成后的 `vcpkg.json` 应提交 Git。

---

## 7. 创建 `CMakePresets.json`

```json
{
  "version": 6,
  "cmakeMinimumRequired": {
    "major": 3,
    "minor": 25,
    "patch": 0
  },
  "configurePresets": [
    {
      "name": "base",
      "hidden": true,
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/${presetName}",
      "toolchainFile": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "cacheVariables": {
        "CMAKE_EXPORT_COMPILE_COMMANDS": true
      }
    },
    {
      "name": "debug",
      "displayName": "Debug",
      "inherits": "base",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug"
      }
    },
    {
      "name": "release",
      "displayName": "Release",
      "inherits": "base",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release"
      }
    }
  ],
  "buildPresets": [
    {
      "name": "debug",
      "configurePreset": "debug"
    },
    {
      "name": "release",
      "configurePreset": "release"
    }
  ],
  "testPresets": [
    {
      "name": "debug",
      "configurePreset": "debug",
      "output": {
        "outputOnFailure": true
      }
    },
    {
      "name": "release",
      "configurePreset": "release",
      "output": {
        "outputOnFailure": true
      }
    }
  ]
}
```

该文件定义了两套独立环境：

```text
build/debug/
build/release/
```

二者共享 Ninja 和 vcpkg Toolchain，但分别使用 Debug 和 Release 配置。

`$env{VCPKG_ROOT}` 读取当前环境变量，因此项目不需要记录某位开发者计算机上的绝对安装路径。

---

## 8. 创建 `.gitignore`

```gitignore
# CMake 构建目录
/build/

# vcpkg 项目依赖安装目录（若生成在项目根目录）
/vcpkg_installed/

# 本地安装产物
/install/

# 本地 CMake 预设
/CMakeUserPresets.json

# 常见编辑器本地文件
/.vs/
/.vscode/
/.idea/
```

`.vscode/` 是否完全忽略取决于团队是否希望共享部分 VS Code 配置。若需要共享，应改为只忽略个人状态文件。

应该提交 Git 的环境描述主要包括：

```text
CMakeLists.txt
CMakePresets.json
vcpkg.json
源代码与测试
```

不应提交的主要内容包括：

```text
build/
vcpkg_installed/
CMakeUserPresets.json
编译生成的可执行文件、库和中间文件
```

---

## 9. 配置项目

打开 **Developer PowerShell for Visual Studio**，进入项目目录，并确认已经设置 `VCPKG_ROOT`。

配置 Debug：

```powershell
cmake --preset debug
```

第一次配置时，CMake 会：

1. 读取 `CMakePresets.json`；
2. 使用 Ninja Generator；
3. 加载 vcpkg Toolchain；
4. 根据 `vcpkg.json` 获取并构建 fmt；
5. 检测 MSVC 工具链；
6. 解析 `CMakeLists.txt`；
7. 在 `build/debug/` 生成构建规则。

依赖第一次可能需要从源码构建。后续构建可以复用 vcpkg Binary Cache。

---

## 10. 构建项目

```powershell
cmake --build --preset debug
```

CMake 会调用 Ninja，Ninja 再根据生成的规则调用 MSVC 编译器和链接器。

---

## 11. 运行测试

```powershell
ctest --preset debug
```

测试失败时，Preset 中的 `outputOnFailure` 会显示失败程序的输出。

---

## 12. 运行程序

使用 Ninja 单配置构建时，程序通常位于对应构建目录：

```powershell
.\build\debug\cpp_demo.exe
```

程序的精确位置仍取决于 CMake Target 属性和 Generator，不应让其他自动化脚本无条件猜测复杂项目的产物路径。安装规则可以提供更稳定的产物布局。

---

## 13. Release 构建

```powershell
cmake --preset release
cmake --build --preset release
ctest --preset release
```

Debug 和 Release 位于不同目录，因此不会共享 CMake Cache 和项目目标文件。

---

## 14. 安装项目

可以指定安装目录：

```powershell
cmake --install build/release --prefix install
```

根据前面的 `install()` 规则，程序会被整理到：

```text
install/bin/
```

`install/` 是发布产物目录，也通常不应提交 Git。

---

## 15. 添加依赖

编辑 `vcpkg.json`，把新依赖加入 `dependencies`，然后重新配置：

```powershell
cmake --preset debug
```

vcpkg 会在 CMake 配置阶段检查 Manifest，并准备新增依赖。随后在 `CMakeLists.txt` 中通过该依赖导出的 CMake Package 和 Target 使用它。

添加依赖时不能只修改 `vcpkg.json`，也不能只写 `find_package()`：前者负责准备依赖，后者负责让构建目标使用依赖。

---

## 16. 更新依赖

更新 vcpkg 的本地注册表后，可以更新 baseline：

```powershell
cd $env:VCPKG_ROOT
git pull
cd <项目目录>
vcpkg x-update-baseline
```

更新后应重新执行：

```powershell
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

依赖升级属于项目变更，应检查 `vcpkg.json` 的差异并通过测试验证，而不是在所有项目中无条件自动升级。

---

## 17. 克隆后恢复环境

一台新机器首先需要安装项目约定的基础工具：

- 编译工具链；
- CMake；
- Ninja；
- vcpkg；
- Git。

随后：

```powershell
git clone <repository>
cd <repository>
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

其中：

- `vcpkg.json` 恢复依赖声明；
- `builtin-baseline` 恢复依赖版本集合；
- `CMakePresets.json` 恢复共享构建参数；
- `CMakeLists.txt` 恢复项目目标和依赖关系；
- vcpkg Toolchain 在配置阶段准备依赖；
- CMake 和 Ninja 调用本机工具链重新生成二进制文件。

这就是 C++ 项目中最接近 Python `uv sync` 的环境恢复过程。

---

## 18. 日常开发命令

首次进入或配置发生变化：

```powershell
cmake --preset debug
```

修改源码后：

```powershell
cmake --build --preset debug
ctest --preset debug
```

发布前：

```powershell
cmake --preset release
cmake --build --preset release
ctest --preset release
cmake --install build/release --prefix install
```

---

---

---

# 环境复现的层级

C++ 项目的“可复现”可以分为三个层级。

## 第一层：依赖声明可复现

提交：

- `vcpkg.json`；
- `builtin-baseline`；
- 必要的自定义 Triplet 或 Registry 配置。

这使项目能够重新解析和构建相同版本集合的第三方依赖。

## 第二层：项目构建配置可复现

进一步提交：

- `CMakeLists.txt`；
- `CMakePresets.json`；
- 测试；
- 工具链最低版本说明。

这使本地开发、IDE 和 CI 可以共享相同的项目构建入口。

## 第三层：完整工具链可复现

进一步固定：

- 操作系统或容器镜像；
- 编译器和 SDK 版本；
- CMake、Ninja 和 vcpkg 版本；
- 构建环境变量；
- CI 执行环境。

对于需要长期维护、正式发布或安全审计的项目，可以使用容器、CI 镜像、开发环境配置或组织内部工具链包进一步约束这些内容。

需要注意：即使输入环境完全固定，不同构建时间、路径或工具行为仍可能影响最终二进制是否逐字节一致。“能够从源码可靠地重新构建”和“生成逐字节相同的二进制文件”是两个不同级别的目标。

---

---

---

# 总结

C++ 环境管理由三部分共同完成：

1. **编译工具链**决定源代码如何转换为特定平台的二进制文件。
2. **CMake 与底层构建器**描述项目目标、生成构建规则并调用工具链。
3. **vcpkg**声明、获取和构建第三方依赖，再通过 CMake 把依赖连接到项目 Target。

对应的项目级信息分别保存在：

| 信息 | 主要记录位置 |
|---|---|
| 项目目标和依赖关系 | `CMakeLists.txt` |
| 共享构建配置 | `CMakePresets.json` |
| 个人本地配置 | `CMakeUserPresets.json` |
| 第三方依赖声明和版本集合 | `vcpkg.json` |
| 本地构建产物和缓存 | `build/` |

推荐的日常工作流是：

```text
准备本机工具链
      ↓
读取 CMake Preset
      ↓
vcpkg 根据 Manifest 准备依赖
      ↓
CMake 配置并生成 Ninja 构建规则
      ↓
Ninja 调用编译器和链接器
      ↓
CTest 运行测试
      ↓
CMake Install 整理发布产物
```

最终，项目仓库保存的是**能够重新构建环境的描述**，而不是某一台计算机上生成的构建目录和二进制文件。

---

# 参考资料

- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/)
- [CMake Presets](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html)
- [vcpkg Overview](https://learn.microsoft.com/vcpkg/get_started/overview)
- [vcpkg Manifest Mode](https://learn.microsoft.com/vcpkg/consume/manifest-mode)
- [vcpkg Versioning](https://learn.microsoft.com/vcpkg/users/versioning)
- [vcpkg 与 CMake 集成](https://learn.microsoft.com/vcpkg/users/buildsystems/cmake-integration)
- [Conan 2 Tutorial](https://docs.conan.io/2/tutorial.html)
- [Conan 与 CMake 集成](https://docs.conan.io/2/integrations/cmake.html)
