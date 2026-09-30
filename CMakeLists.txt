cmake_minimum_required(VERSION 3.16)

project(sdf_dcd_demo LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Demo sources
file(GLOB SOURCES CONFIGURE_DEPENDS "source/*.cpp" "source/*.h")

# Dear ImGui with its SDL2 and OpenGL3 backends
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)

if(EMSCRIPTEN)
    # WebGL2 build with Emscripten's SDL2 port. The OBJ assets are embedded and everything is
    # packed into a single index.html.
    target_compile_options(${PROJECT_NAME} PRIVATE -sUSE_SDL=2)
    target_link_options(${PROJECT_NAME} PRIVATE
        -sUSE_SDL=2
        -sMIN_WEBGL_VERSION=2
        -sMAX_WEBGL_VERSION=2
        -sALLOW_MEMORY_GROWTH=1
        -sINITIAL_MEMORY=64MB
        -sFORCE_FILESYSTEM=1
        -sEXPORTED_RUNTIME_METHODS=ccall,FS
        -sENVIRONMENT=web
        -sSINGLE_FILE=1
        --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/source/shell.html
        --embed-file ${CMAKE_CURRENT_SOURCE_DIR}/assets@/assets)
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "index" SUFFIX ".html")
else()
    # SDL2 is built from the submodule and linked statically
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    add_subdirectory(external/SDL EXCLUDE_FROM_ALL)
    find_package(OpenGL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main SDL2::SDL2-static OpenGL::GL)

    if(WIN32)
        target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
        target_link_libraries(${PROJECT_NAME} PRIVATE comdlg32)
    endif()
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /fp:fast)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
    endif()

    # Copy the assets next to the executable
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets)
endif()
