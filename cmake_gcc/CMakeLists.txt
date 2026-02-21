# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	sgp40_air_beacon
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_sgp40_air_beacon target,
# which contains the content of the SLC project
include(sgp40_air_beacon.cmake)

add_executable(sgp40_air_beacon
    # Add additional sources here
)

target_include_directories(sgp40_air_beacon PUBLIC
    # Add additional include paths here
)

target_compile_definitions(sgp40_air_beacon PUBLIC
    # Add additional macros here
)

target_compile_options(sgp40_air_beacon PUBLIC
    # Set additional compiler flags here
)

target_link_options(sgp40_air_beacon PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(sgp40_air_beacon PRIVATE
    slc
)

# Include managed project content if available
include(sgp40_air_beacon_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(sgp40_air_beacon PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET sgp40_air_beacon
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:sgp40_air_beacon>" "$<TARGET_FILE_DIR:sgp40_air_beacon>/$<TARGET_FILE_BASE_NAME:sgp40_air_beacon>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:sgp40_air_beacon>" "$<TARGET_FILE_DIR:sgp40_air_beacon>/$<TARGET_FILE_BASE_NAME:sgp40_air_beacon>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:sgp40_air_beacon>" "$<TARGET_FILE_DIR:sgp40_air_beacon>/$<TARGET_FILE_BASE_NAME:sgp40_air_beacon>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET sgp40_air_beacon
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()