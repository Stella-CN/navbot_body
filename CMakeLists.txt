cmake_minimum_required(VERSION 3.22)

# Select cross compilation before project() can probe a host compiler.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "Arm GNU toolchain")
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(navbot_body LANGUAGES C CXX ASM)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR "navbot_body requires the Arm bare-metal toolchain")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-mthumb)
add_executable(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
target_link_options(${PROJECT_NAME} PRIVATE -mthumb)
add_subdirectory(cmake/stm32cubemx)
# FreeRTOSConfig.h references the BSP panic declaration. Propagate its include
# directory to generated application, HAL and FreeRTOS object targets alike.
target_include_directories(stm32cubemx INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Applications"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/C30D"
)
target_sources(${PROJECT_NAME} PRIVATE
    Applications/motor.cpp
    Applications/wheel_motors.cpp
    Applications/link_protocol.cpp
    Applications/chassis_link.cpp
    Applications/attitude.cpp
    Applications/app_tasks.cpp
    Applications/imu_diagnostics.cpp
    BSP/C30D/c30d_soft_i2c.c
    BSP/C30D/c30d_imu.c
    BSP/C30D/c30d_oled.c
    BSP/C30D/c30d_safety.c
    BSP/C30D/c30d_motor.c
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    "$<$<CONFIG:Debug>:C30D_IMU_TEST_HOOKS>"
)
target_compile_options(${PROJECT_NAME} PRIVATE
    "$<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-Wextra;-Wpedantic>"
)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
