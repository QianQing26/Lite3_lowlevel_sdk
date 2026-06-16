cmake_minimum_required(VERSION 3.10)

project(rl_controller)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)

# ========== 控制选项 ==========
set(BUILD_PLATFORM "arm" CACHE STRING "select build cpu type: x86 or arm")
option(SEND_REMOTE "Auto-scp binary to robot after build" OFF)
option(USE_MOLD_LINKER "Use mold linker for faster linking" OFF)

if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
else()
  message("this is x86 platform")
endif()
set(CMAKE_CXX_STANDARD 17)

# ========== 显示参数 ==========
message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")

# 路径设置
get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

# ========== Include 头文件路径 ==========
include_directories(
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/Lite3_MotionSDK/include/common
  ${THIRD_PARTY}/Lite3_MotionSDK/include
  communication
  types
  utils
  interface/robot
  interface/user_command
  state_machine
)

# ========== 添加 interface 子模块 ==========
add_subdirectory(interface)

# ========== 快速链接器 (可选) ==========
if(USE_MOLD_LINKER)
    find_program(MOLD_EXECUTABLE mold)
    if(MOLD_EXECUTABLE)
        add_link_options(-fuse-ld=mold)
        message("Using mold linker: ${MOLD_EXECUTABLE}")
    else()
        message(WARNING "USE_MOLD_LINKER=ON but mold not found; using default linker")
    endif()
endif()

# ========== 收集源文件 ==========
file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

# ========== 链接基本库 ==========
target_link_libraries(rl_deploy interface -lpthread -lm -lrt -ldl -lstdc++)

# ========== Lite3 MotionSDK 动态库 ==========
if (BUILD_PLATFORM STREQUAL arm)
  target_link_libraries(rl_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so
      pthread
      rt
  )
else()
  target_link_libraries(rl_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so
      pthread
      rt
  )
endif()

# ========== 远程部署脚本 ==========
if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()

