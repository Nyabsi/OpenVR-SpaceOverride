cmake_minimum_required(VERSION 3.26)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(openvr_spaceoverride)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)
find_package(OpenVR REQUIRED)

# Include rest of the CMake bindings
add_subdirectory(3rdparty)

set(SOURCES_OVERLAY
    "overlay/Main.cpp"
    "overlay/Calibration.cpp"
    "overlay/Configuration.cpp"
    "overlay/IPCClient.cpp"
    "overlay/UserInterface.cpp"
    "overlay/DroidSans_Font.cpp"
)

add_executable(openvr_spaceoverride ${SOURCES_OVERLAY})

if (UNIX AND NOT APPLE)
    set_target_properties(openvr_spaceoverride PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN"
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )
endif()

target_include_directories(openvr_spaceoverride PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/overlay)
target_include_directories(openvr_spaceoverride PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/shared)

# Windows executable icon
if (WIN32)
    target_sources(openvr_spaceoverride PRIVATE "resources/winicon.rc")
endif()

target_compile_features(openvr_spaceoverride PUBLIC cxx_std_20)

target_link_libraries(openvr_spaceoverride
    PRIVATE
        OpenVR::API
        ImGui
        GL3W
        GLFW
        Eigen
)

target_compile_options(openvr_spaceoverride PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Werror>
)


set(SOURCES_DRIVER
    "driver/Main.cpp"
    "driver/Hooking.cpp"
    "driver/InterfaceHookInjector.cpp"
    "driver/IPCServer.cpp"
    "driver/Logging.cpp"
    "driver/TrackedDeviceProvider.cpp"
)

add_library(driver_spaceoverride SHARED ${SOURCES_DRIVER})

set_target_properties(driver_spaceoverride PROPERTIES PREFIX "")
target_include_directories(driver_spaceoverride PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/driver)
target_include_directories(driver_spaceoverride PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/shared)
target_compile_features(driver_spaceoverride PUBLIC cxx_std_20)

target_link_libraries(driver_spaceoverride
    PRIVATE
        OpenVR::API
        MinHook
)

target_compile_options(driver_spaceoverride PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Werror>
)

set(CUSTOM_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin")

add_custom_target(openvr_spaceoverride_resources)

if (WIN32)
    set(OPENVR_RUNTIME_BINARY "$<TARGET_FILE:OpenVR::API>")
else()
    set(OPENVR_RUNTIME_BINARY "${OpenVR_BINARY}")
endif()

add_custom_command(
    TARGET openvr_spaceoverride_resources POST_BUILD
    COMMENT "Copying Resources"
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources ${CUSTOM_OUTPUT_DIR}
)

add_dependencies(openvr_spaceoverride openvr_spaceoverride_resources)

add_custom_command(
    TARGET openvr_spaceoverride POST_BUILD
    COMMENT "Copying Executable and DLLs"
    COMMAND ${CMAKE_COMMAND} -E copy -t ${CUSTOM_OUTPUT_DIR} $<TARGET_FILE:openvr_spaceoverride>
    COMMAND ${CMAKE_COMMAND} -E copy -t ${CUSTOM_OUTPUT_DIR} ${OPENVR_RUNTIME_BINARY}
    COMMAND_EXPAND_LISTS
)

if (WIN32)
    set(DRIVER_PLATFORM "win64")
else()
    set(DRIVER_PLATFORM "linux64")
endif()

add_custom_command(
    TARGET driver_spaceoverride POST_BUILD
    COMMENT "Copying Driver Resources and Binary"
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/dev-resources/driver ${CUSTOM_OUTPUT_DIR}/driver
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CUSTOM_OUTPUT_DIR}/driver/spaceoverride/bin/${DRIVER_PLATFORM}
    COMMAND ${CMAKE_COMMAND} -E copy -t ${CUSTOM_OUTPUT_DIR}/driver/spaceoverride/bin/${DRIVER_PLATFORM} $<TARGET_FILE:driver_spaceoverride>
)