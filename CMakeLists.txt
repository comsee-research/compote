cmake_minimum_required(VERSION 2.8)

get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${ProjectId})

message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
message("COMPOTE: Calibration Of Multi-focus PlenOpTic camEra")
message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

add_subdirectory(src/calibrate)
add_subdirectory(src/precalibrate)
add_subdirectory(src/detect)
add_subdirectory(src/extrinsics)

add_subdirectory(src/blurcalib)
add_subdirectory(src/invdistortioncalib)


