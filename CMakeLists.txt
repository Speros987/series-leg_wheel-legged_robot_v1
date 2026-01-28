cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME series-leg_wheel-legged_robot_v1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/Balance_contral/Virtual_Chassis_Control.c
        User/Balance_contral/Virtual_Chassis_Control.h
        User/Balance_contral/Init.c
        User/Balance_contral/Init.h
        User/Balance_contral/state_define.c
        User/Balance_contral/state_define.h
        User/Balance_contral/Task_define.c
        User/Balance_contral/Task_define.h
        User/Balance_contral/VMC.c
        User/Balance_contral/VMC.h
        User/Balance_contral/Out_of_Contral_Detection.c
        User/Balance_contral/Out_of_Contral_Detection.h
        User/Balance_contral/Upstairs.c
        User/Balance_contral/Upstairs.h
        User/Balance_contral/Jump.c
        User/Balance_contral/Jump.h
        User/Balance_contral/Emergency.c
        User/Balance_contral/Emergency.h
        User/Balance_contral/Remote_Contral.c
        User/Balance_contral/Remote_Contral.h
        User/Balance_contral/speed.c
        User/Balance_contral/speed.h
        User/Balance_contral/Main_Output.c
        User/Balance_contral/Main_Output.h
        User/Balance_contral/Lie_Down.c
        User/Balance_contral/Lie_Down.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS-DSP-main/Source bin_dsp)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

        User/CAN/DM_can_IO.c
        User/CAN/fliter.c

        User/Balance_contral/callback.c
        User/Balance_contral/panel.c
        User/Balance_contral/my_maths.c
        User/Balance_contral/state_define.c
        User/Balance_contral/Emergency.c
        User/Balance_contral/Init.c
        User/Balance_contral/Jump.c
        User/Balance_contral/Virtual_Chassis_Control.c
        User/Balance_contral/Remote_Contral.c
        User/Balance_contral/Task_define.c
        User/Balance_contral/Upstairs.c
        User/Balance_contral/VMC.c
        User/Balance_contral/Out_of_Contral_Detection.c
        User/Balance_contral/Main_Output.c

        User/Device/Algorithm/controller.c
        User/Device/Algorithm/algorithm.c
        User/Device/Algorithm/kalman_filter.c
        User/Device/Algorithm/pid.c
        User/Device/Algorithm/user_lib.c
        User/Device/Algorithm/QuaternionEKF.c

        User/Device/BMI088/bsp_dwt.c
        User/Device/BMI088/BMI088driver.c
        User/Device/BMI088/BMI088Middleware.c

        User/Device/BMI088/App/imu_temp_ctrl.c

        User/Device/Mahony/MahonyAHRS.c

        User/Device/HT-10A/remoter.c
        User/Device/Breezer/buzzer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        User
            User/CAN
            User/Balance_contral
                User/Balance_contral/output
            User/Device
                User/Device/Algorithm
                User/Device/BMI088
                    User/Device/BMI088/App
                    User/Device/BMI088/inc
                User/Device/Mahony
                User/Device/Remote_Contraler
                User/Device/HT-10A
                User/Device/Breezer

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    # Add user defined libraries
)
