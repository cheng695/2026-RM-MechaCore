#!/bin/bash
# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# 从当前目录开始，向上查找直到找到 CMakeLists.txt 文件，确定项目根目录
PROJECT_DIR=$SCRIPT_DIR
while [ ! -f "$PROJECT_DIR/CMakeLists.txt" ]; do
PROJECT_DIR=$(dirname "$PROJECT_DIR")
done
# 获取项目根目录名
PROJECT_NAME=$(basename "$PROJECT_DIR")
echo "Project root directory name: $PROJECT_NAME"

# 创建并进入构建目录
BUILD_DIR="$PROJECT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 运行 Cmake 配置和编译项目
echo "正在运行CMake配置..."
if ! cmake ..; then
    echo "错误：CMake配置失败，取消烧录操作"
    exit 1
fi

echo "正在编译项目..."
if ! make -j 16; then
    echo "错误：编译失败，取消烧录操作"
    exit 1
fi

echo "编译成功完成"

# 查找ELF文件，找不到则尝试使用项目名，最后尝试查找任意.elf文件
if [ -f "$BUILD_DIR/${TARGET_NAME}.elf" ]; then
    ELF_FILE="$BUILD_DIR/${TARGET_NAME}.elf"
elif [ -f "$BUILD_DIR/${PROJECT_NAME}.elf" ]; then
    ELF_FILE="$BUILD_DIR/${PROJECT_NAME}.elf"
    TARGET_NAME=$PROJECT_NAME
else
    # 查找任何.elf文件
    ELF_FILE=$(find "$BUILD_DIR" -maxdepth 1 -name "*.elf" | head -n 1)
    if [ -z "$ELF_FILE" ]; then
        echo "Error: No ELF file found in build directory. Compilation might have failed."
        exit 1
    fi
    TARGET_NAME=$(basename "$ELF_FILE" .elf)
fi

BIN_FILE="$BUILD_DIR/${TARGET_NAME}.bin"
HEX_FILE="$BUILD_DIR/${TARGET_NAME}.hex"

echo "Using target: ${TARGET_NAME}"
echo "ELF file: ${ELF_FILE}"

# 检查 ELF 文件是否成功生成
if [ -f "$ELF_FILE" ]; then
# 将 ELF 文件转换为 BIN 文件和 HEX 文件
arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
arm-none-eabi-objcopy -O ihex "$ELF_FILE" "$HEX_FILE"
echo "Conversion to BIN and HEX completed"
else
echo "Error: ELF file not found. Compilation might have failed."
exit 1
fi

# 执行 OpenOCD 进行烧录
cd "$PROJECT_DIR"

# daplink
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "gdb_port 3334" -c "program build/${TARGET_NAME}.elf verify reset exit"


# # stlink
# echo "使用stlink烧录"
# openocd -f interface/stlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "gdb_port 3334" -c "program build/${TARGET_NAME}.elf verify reset exit"

# # jlink
# # 添加JLink到PATH环境变量（Bash语法）
# export PATH="$PATH:/d/SEGGER/JLink_V722b"
# echo start...

# # 创建日志目录
# LOG_DIR="$BUILD_DIR/log"
# mkdir -p "$LOG_DIR"

# # 生成带时间戳的日志文件名
# TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
# LOG_FILE="$LOG_DIR/jlink_flash_${TIMESTAMP}.log"

# # 创建临时 JLink 命令文件并进行烧录
# cd "$BUILD_DIR"

# # 方案1：标准复位流程（推荐先尝试）
# cat > download.jlink << EOF
# log log/jlink_flash_${TIMESTAMP}.log
# h
# loadfile ${TARGET_NAME}.hex
# r
# g
# qc
# EOF

# 方案2：如果方案1不行，取消注释下面的代码，注释掉方案1
# cat > download.jlink << EOF
# log log/jlink_flash_${TIMESTAMP}.log
# h
# erase
# loadfile ${TARGET_NAME}.hex
# r
# g
# qc
# EOF

# 方案3：使用BIN文件（如果HEX有问题）
# cat > download.jlink << EOF
# log log/jlink_flash_${TIMESTAMP}.log
# h
# loadbin ${TARGET_NAME}.HEX 0x08000000
# r
# g
# qc
# EOF

# # 使用临时文件进行烧录
# JLink.exe -autoconnect 1 -device STM32F407IG -if swd -speed 5000 -CommanderScript download.jlink