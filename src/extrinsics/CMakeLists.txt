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
)

message(${LIBPLENO_LIBRARIES})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBPLENO_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

##################################################
##################################################
add_executable(extrinsics src/extrinsics.cpp ${MULTIFOCUS_SRCS})
target_include_directories(extrinsics PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(extrinsics ${MULTIFOCUS_LIBS})

##################################################
##################################################
set(COMPILE_LEGACY_EVAL FALSE CACHE BOOL "Set to TRUE to enable compilation of legacy evaluation apps")
if (COMPILE_LEGACY_EVAL)

	set(MULTIFOCUS_STATS_SRCS 
		src/stats/utils.cpp
	)
	##################################################
	##################################################
	add_executable(linear_evaluation src/stats/linear_evaluation.cpp ${MULTIFOCUS_STATS_SRCS})
	target_include_directories(linear_evaluation PRIVATE ${MULTIFOCUS_INCDIRS})
	target_link_libraries(linear_evaluation ${MULTIFOCUS_LIBS})

	##################################################
	##################################################
	add_executable(linear_raytrix_evaluation src/stats/linear_raytrix_evaluation.cpp ${MULTIFOCUS_STATS_SRCS})
	target_include_directories(linear_raytrix_evaluation PRIVATE ${MULTIFOCUS_INCDIRS})
	target_link_libraries(linear_raytrix_evaluation ${MULTIFOCUS_LIBS})
endif (COMPILE_LEGACY_EVAL)

