cmake_minimum_required(VERSION 3.21)
project(Mujoco_controller_playground)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

message("Cmake source directroy:" + ${CMAKE_CURRENT_SOURCE_DIR})
message("Cmake runtime directroy:" + ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    conan_basic_setup()
else()
    message(WARNING "conanbuildinfo.cmake does not exist, you need to run INSTALL first")
endif()

file(GLOB LIB_MUJOCOL ${CMAKE_CURRENT_SOURCE_DIR}/Mujoco/mujoco210.lib)
file(GLOB LIB_MUJOCOD ${CMAKE_CURRENT_SOURCE_DIR}/Mujoco/mujoco210.dll)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/GLFW)

include_directories(PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/GLFW
                    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Mujoco
                    )

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Mujoco)

add_executable(${PROJECT_NAME} main.cpp
                                Utility/MujocoController/MujocoUI.cpp Utility/MujocoController/MujocoUI.h
                                Utility/MujocoController/MujocoController.cpp Utility/MujocoController/MujocoController.h
                                ikfast/ikfastPanda.cpp ikfast/ikfast.h
                                ikValidater/ikValidater.cpp ikValidater/ikValidater.h
                                iLQR/iLQR_funcs.cpp iLQR/iLQR_funcs.h)

target_link_libraries(${PROJECT_NAME} glfw ${LIB_MUJOCOL} ${LIB_MUJOCOD})
