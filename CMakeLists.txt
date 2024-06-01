cmake_minimum_required(VERSION 3.10)
project(AVR-USART-WheelServo C)


# 設定 MCU 和編譯選項
set(MCU atmega328p)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-mmcu=${MCU} -Os")

# Find all .c files in the src directory
file(GLOB SOURCES "src/*.c")

# Add executable with all found .c files
add_executable(main.elf ${SOURCES})

# 添加自訂命令來生成 HEX 文件
add_custom_command(TARGET main.elf POST_BUILD
    COMMAND avr-objcopy -O ihex $<TARGET_FILE:main.elf> main.hex
    COMMENT "Building HEX file"
)
