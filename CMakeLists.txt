cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (cerebellum)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE ON)

###############################################################################

set (CMAKE_C_STANDARD 99)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")
set (CMAKE_HEADERS_OUTPUT_DIRECTORY "include")
#set (CMAKE_INSTALL_PREFIX ".")

set (CMAKE_BUILD_TYPE Release)
#set (CMAKE_BUILD_TYPE Debug)
#set (CMAKE_BUILD_TYPE RelWithDebInfo)
#set (CMAKE_BUILD_TYPE MinSizeRel)

###############################################################################
# Library
###############################################################################

add_library(cerebellum INTERFACE)
target_include_directories(cerebellum
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
        $<INSTALL_INTERFACE:include>
)

###############################################################################
# App
###############################################################################

add_executable(app src/main.cxx)
target_link_libraries(app cerebellum)

if(MSVC)
	# using MSVC
	target_compile_options(app PRIVATE /EHsc /W3 /WX)
else()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		# using Clang
		target_compile_options(app PRIVATE -Werror -Wall -Wextra -pedantic)
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		# using GCC
		target_compile_options(app PRIVATE -Werror -Wall -Wextra -pedantic)
	endif()
endif()

###############################################################################
# Game
###############################################################################

add_executable(game src/ccc.cxx)
target_link_libraries(game cerebellum)

if(MSVC)
	# using MSVC
	target_compile_options(game PRIVATE /EHsc /W3 /WX)
else()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		# using Clang
		target_compile_options(game PRIVATE -Werror -Wall -Wextra -pedantic)
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		# using GCC
		target_compile_options(game PRIVATE -Werror -Wall -Wextra -pedantic)
	endif()
endif()

###############################################################################
# Test
###############################################################################

add_library(catch2 INTERFACE)
target_include_directories(catch2
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/test>
)

add_executable(test_cb src/test/test.cxx)
target_link_libraries(test_cb cerebellum)
target_link_libraries(test_cb catch2)

if(MSVC)
	# using MSVC
	target_compile_options(test_cb PRIVATE /EHsc /W3 /WX)
else()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		# using Clang
		target_compile_options(test_cb PRIVATE -Werror -Wall -Wextra -pedantic)
	elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		# using GCC
		target_compile_options(test_cb PRIVATE -Werror -Wall -Wextra -pedantic)
	endif()
endif()

###############################################################################
