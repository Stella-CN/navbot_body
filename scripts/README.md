# 开发脚本

使用 Bash 3.2+，适用于 macOS/Linux。路径按脚本位置解析，可从任意工作目录调用。参考 vehicle-firmware 的脚本用途，保持 navbot_body 的 CMake预设和 Applications/BSP 分层。脚本没有下载、复位或探针操作。

| 脚本 | 用法 | 行为 |
| --- | --- | --- |
| `doctor.sh` | `./scripts/doctor.sh` | 检查构建工具及GCC目标，列出可选工具；不探测硬件 |
| `build.sh` | `./scripts/build.sh [Debug\|Release] [jobs]` | 配置、编译、链接，导出ELF/BIN/HEX和大小报告 |
| `clean.sh` | `./scripts/clean.sh [Debug\|Release]` | 调用CMake clean并删除本脚本导出的报告/镜像，保留配置与日志 |
| `check.sh` | `./scripts/check.sh` | Bash语法检查及固件静态断言，不编译 |
| `format.sh` | `./scripts/format.sh [--check\|--write]` | 默认只检查；`--write`格式化Applications/BSP中的C/C++代码 |
| `symbols.sh` | `./scripts/symbols.sh [Debug\|Release]` | 对已有ELF导出按地址排序的已定义符号表及大小报告 |

所有入口支持 `--help`。`common.sh` 为共用路径、依赖、预设与缓存校验，不作为独立命令。

## 常用流程

```sh
./scripts/doctor.sh
./scripts/check.sh
./scripts/build.sh Debug 4
./scripts/symbols.sh Debug
```

`build.sh` 默认Debug，jobs取第二参数、环境变量 `CMAKE_BUILD_PARALLEL_LEVEL` 或默认4；只接受正整数。配置/编译错误经 `tee` 记录后仍返回非零状态，不继续导出，也不会打印构建成功。构建失败后目录可能保留上次成功产物，应以退出码和本次日志为准。

产物均位于 `build/<预设>/`，已通过 `.gitignore` 排除：

- `navbot_body.elf`、`navbot_body.bin`、`navbot_body.hex`
- `navbot_body.map`、`navbot_body.size.txt`
- `navbot_body.symbols.txt`（显式调用symbols后生成）
- `configure.log`、`build.log`、`compile_commands.json`

`clean.sh` 不递归删除目录，只清理对应预设的构建产物；配置尚未生成时直接成功返回。已有CMake缓存必须归属于当前源码目录，否则构建/清理均停止，避免误用其他工程缓存。不要同时对同一预设执行构建与清理。

格式化需要支持 `--dry-run --Werror` 的 `clang-format`；不处理Core、Drivers或Middlewares。默认模式和 `--check` 只检查，不修改文件；`--write` 才写入。发现格式问题或工具缺失时返回非零，检查会在首个失败文件处停止；返回1不能直接等同于工具安装失败。可选GDB只用于后续调试；脚本不调用它或OpenOCD。

VS Code的默认构建任务已调用 `build.sh Debug`，另有静态检查、清理、符号表、诊断、格式检查任务。启动调试仍须单独操作，本轮不包含调试或下载。

参考：[CMake命令行与构建预设](https://cmake.org/cmake/help/latest/manual/cmake.1.html)、[Bash pipefail](https://www.gnu.org/software/bash/manual/html_node/Pipelines.html)。
