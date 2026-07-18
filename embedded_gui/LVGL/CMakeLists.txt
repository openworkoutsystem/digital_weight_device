# Migrated SquareLine UI built as an LVGL Editor project

# Only set project if this is the top-level CMakeLists.txt
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.10)
  project(MigratedUI)
  set(IS_TOP_LEVEL TRUE)
else()
  set(IS_TOP_LEVEL FALSE)
endif()

# Include the generated list of .c files
include(${CMAKE_CURRENT_LIST_DIR}/file_list_gen.cmake)

# Create the UI sources as a library
add_library(lib-ui ${PROJECT_SOURCES})

# Ensure headers resolve from this migrated project
target_include_directories(lib-ui PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Configuration branching for Editor preview vs. target
if (LV_EDITOR_PREVIEW)
    # Settings specific to the LVGL Editor Preview can be added here
else ()
    # Configure for your embedded target here (include directories, defines, etc.)
    # e.g., target_include_directories(lib-ui PUBLIC <path-to-lvgl>)
endif ()

if (IS_TOP_LEVEL)
  # Optional: top-level behaviors (e.g., add_subdirectory for lvgl, demo app, etc.)
else()
  # Behaviors when included as a subdirectory
endif()
