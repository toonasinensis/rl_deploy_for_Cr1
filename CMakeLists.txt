cmake_minimum_required(VERSION 3.10)

project(rl_deploy)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
set(BUILD_PLATFORM "arm" CACHE STRING "select build cpu type")
option(BUILD SIM OFF)
option(SEND_REMOTE OFF)
if (BUILD_PLATFORM STREQUAL arm)
    message("this is arm platform")
    # set(BUILD_SIM OFF)
    set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
else ()
    message("this is x86 platform")
endif ()
set(CMAKE_CXX_STANDARD 17)

# include(config.cmake)
if (SEND_REMOTE)
    set(BUILD_SIM OFF)
endif ()

if (BUILD_SIM)
# 全局定义宏（对所有目标生效）
add_definitions(-DSIMULATION_MODE)
# 或赋值：add_definitions(-DSIMULATION_MODE=1)
endif()

message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")

get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

set(DR_LIB_PATH ${THIRD_PARTY}/dr_lib/${BUILD_PLATFORM})
# message(${DR_LIB_PATH})
include_directories(
        ${DR_LIB_PATH}/drdds/include/dridl
        ${DR_LIB_PATH}/drdds/include
        ${DR_LIB_PATH}/drdds/

        ${DR_LIB_PATH}/log4cplus/include/
        ${DR_LIB_PATH}/log4cplus/include/log4cplus_dr/
)
set(CMAKE_PREFIX_PATH ${DR_LIB_PATH}/drdds/lib/)

# ## 2. set all include folders
# find_package(fastcdr REQUIRED)
# find_package(fastrtps REQUIRED)
link_directories(
        ${DR_LIB_PATH}/drdds/
        ${DR_LIB_PATH}/drdds/lib
        ${DR_LIB_PATH}/log4cplus/lib
)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfastcdr.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfastrtps.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfoonathan_memory-0.7.3.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libssl.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libcrypto.so)
# list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libtinyxml2.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/libdrdds.so)
list(APPEND dr_lib ${DR_LIB_PATH}/log4cplus/lib/liblog4cplus.so)

message("${dr_lib}")

include_directories(
        ${THIRD_PARTY}/eigen/
        ${THIRD_PARTY}/gamepad/include
        ${THIRD_PARTY}/x30_motion_sdk/include
        ${THIRD_PARTY}/Lite3_MotionSDK/include
        types
        utils
        interface/robot
        interface/user_command
        state_machine
        run_policy
        ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

link_directories(
        ${DR_LIB_PATH}/drdds/lib/
)


add_subdirectory(interface)

if (BUILD_SIM)
    add_definitions(-DUSE_SIMULATION)

    #cd /*path to current foler path*/
    #gedit raisim_path.txt
    #add path to raisimConfig.cmake ///home/mzw/raisim_workspace/raisim_build/lib/cmake/raisim

    # file(READ interface/robot/simulation/raisim_path.txt RAISIM_PATH)
    # string(REGEX REPLACE "\n" ";" RAISIM_PATH "${RAISIM_PATH}")
    # message("${RAISIM_PATH}")

    #  set(raisim_DIR "/home/ysc/raisim_build/lib/cmake/raisim/")
    #  find_package(raisim CONFIG REQUIRED)
endif ()

if (SEND_REMOTE)
    set(PATH_LIB "../third_party")
    set(DR_PATH_LIB ${PATH_LIB}/dr_lib/${BUILD_PLATFORM})
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH
            ${PATH_LIB}/onnxruntime/arm/lib/
            ${PATH_LIB}/onnxruntime/
            ${DR_PATH_LIB}/drdds/
            ${DR_PATH_LIB}/drdds/lib
            ${DR_PATH_LIB}/log4cplus/lib
    )
endif()


file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC} ${RUN_POLICY_SRC} ${RECORDER_PATH})

if (BUILD_PLATFORM STREQUAL arm)
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")
else ()
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")
endif ()

message("${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/")
target_link_libraries(rl_deploy interface -lpthread -lm -lrt -ldl -lstdc++ ${dr_lib} onnxruntime)

if (BUILD_SIM)
    message("build simulation")
    #  target_link_libraries(rl_deploy raisim::raisim)
endif ()

 set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
 set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
 set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
 set(CMAKE_INSTALL_RPATH
   ${DR_LIB_PATH}/drdds/lib/
   ${DR_LIB_PATH}/drdds/
   ${DR_LIB_PATH}/log4cplus/lib/
   ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
   ../lib/onnxruntime/)


if (SEND_REMOTE)
    add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
    add_custom_command(
            TARGET push_to_robot POST_BUILD
            COMMAND ./scripts/sftp_to_remote.sh
            WORKING_DIRECTORY ..
            COMMENT "run sftp_to_remote.sh"
    )
endif ()

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION 1.0)
set(CPACK_PACKAGE_NAME cr1b_deploy)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY " Project CR1B0")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.v${CPACK_PACKAGE_VERSION}.${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_PACKAGE_CONTACT "xqp")
message(${CPACK_PACKAGE_FILE_NAME})
set(CPACK_SKIP_PREINSTALL_COMMAND ON) 

install(FILES build/rl_deploy
        DESTINATION /home/user/rl_deploy/bin/)

install(DIRECTORY data/
        DESTINATION /home/user/rl_deploy/data/)   

install(DIRECTORY policy/
        DESTINATION /home/user/rl_deploy/policy/)      

install(DIRECTORY third_party/
        DESTINATION /home/user/rl_deploy/third_party/)

include(CPack)