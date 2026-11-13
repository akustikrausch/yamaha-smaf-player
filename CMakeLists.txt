# yamaha smaf (.mmf) player, the polyphonic-ringtone engine.
# qt-free, dependency-free C++20. link the `smaf` static lib into anything, or
# build the example CLI that renders a .mmf to a wav.
cmake_minimum_required(VERSION 3.16)
project(yamaha_smaf_player LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# the engine: four translation units plus a header-only codec, no third-party anything.
add_library(smaf STATIC
    src/smaf_file.cpp
    src/ma_fm_core.cpp
    src/smaf_voice.cpp
    src/ma_player.cpp
)
target_include_directories(smaf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# the example CLI, built only when this is the top-level project.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_executable(smaf-render example/smaf-render.cpp)
    target_link_libraries(smaf-render PRIVATE smaf)

    include(CTest)
    if(BUILD_TESTING)
        add_executable(smaf-ma7-test tests/ma7.cpp)
        target_link_libraries(smaf-ma7-test PRIVATE smaf)
        add_test(NAME ma7 COMMAND smaf-ma7-test)
    endif()
endif()
