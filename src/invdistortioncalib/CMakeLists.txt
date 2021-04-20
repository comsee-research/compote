cmake_minimum_required(VERSION 2.8)

get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${ProjectId})

message("-----------------------------------------------------------------------------------------")
message("${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

set(CMAKE_CXX_STANDARD 17)

find_package(libpleno REQUIRED)
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
add_definitions(-O3)

##LINK LIBRARIES
set(MULTIFOCUS_LIBS
	${LIBPLENO_LIBRARIES}
	${Boost_LIBRARIES}
)

##INCLUDE DIRECTORIES
set(MULTIFOCUS_INCDIRS "src")

##SOURCES
set(MULTIFOCUS_SRCS
	src/utils.cpp
	src/invdistortion.cpp
)

message(${LIBPLENO_LIBRARIES})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBPLENO_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

##################################################
##################################################
add_executable(${ProjectId} ${MULTIFOCUS_SRCS})
target_include_directories(${ProjectId} PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(${ProjectId} ${MULTIFOCUS_LIBS})
