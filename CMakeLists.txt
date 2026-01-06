# Copyright (c) 2024 Semtech Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# # Print ZEPHYR_BASE
# message(STATUS "ZEPHYR_BASE: $ENV{ZEPHYR_BASE}")

# # Print CMake source directory (root of your project)
# message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

# # Print current CMake file directory
# message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")

# Tell Zephyr where to find extra modules
set(ZEPHYR_EXTRA_MODULES
    "$ENV{ZEPHYR_BASE}/../modules/lib/usp"
    "$ENV{ZEPHYR_BASE}/../modules/lib/usp_zephyr"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lorawan_periodical_uplink)

target_sources(app PRIVATE
  src/main.c
)

# Apply the same compile definitions as LBM library
# Link against the INTERFACE library that contains all LBM compile definitions
target_link_libraries(app PRIVATE lbm_compile_definitions)
