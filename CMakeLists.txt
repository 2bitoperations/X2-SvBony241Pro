cmake_minimum_required(VERSION 3.10)

project(x2svbony241pro)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(SOURCES
    main.cpp
    x2svbony241pro.cpp
)

# ---------------------------------------------------------------------------
# Shared library target
# ---------------------------------------------------------------------------
add_library(x2svbony241pro SHARED ${SOURCES})

# ---------------------------------------------------------------------------
# Include directories
# TheSkyX X2 licensed interfaces live one level up in X2-Examples.
# ---------------------------------------------------------------------------
target_include_directories(x2svbony241pro PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../X2-Examples/licensedinterfaces
)

# ---------------------------------------------------------------------------
# Output name (without platform prefix/suffix — cmake adds those)
# ---------------------------------------------------------------------------
set_target_properties(x2svbony241pro PROPERTIES
    OUTPUT_NAME "x2svbony241pro"
)

# ---------------------------------------------------------------------------
# Platform-specific settings
# ---------------------------------------------------------------------------
if(APPLE)
    # macOS: build as a plain dynamic library (not a framework bundle).
    set_target_properties(x2svbony241pro PROPERTIES
        BUNDLE FALSE
        SUFFIX ".dylib"
    )
elseif(WIN32)
    set_target_properties(x2svbony241pro PROPERTIES
        SUFFIX ".dll"
    )
else()
    # Linux / other UNIX
    set_target_properties(x2svbony241pro PROPERTIES
        SUFFIX ".so"
    )
endif()

# ---------------------------------------------------------------------------
# Compiler warnings
# ---------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(x2svbony241pro PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# ---------------------------------------------------------------------------
# Install rule (optional — adjust DESTINATION to match your TSX plugin path)
# ---------------------------------------------------------------------------
# install(TARGETS x2svbony241pro
#     LIBRARY DESTINATION /Applications/TheSkyX Professional Edition.app/Contents/Resources/Common/Plugins/PowerControlBox
#     RUNTIME DESTINATION /Applications/TheSkyX Professional Edition.app/Contents/Resources/Common/Plugins/PowerControlBox
# )
