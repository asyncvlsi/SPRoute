cmake_minimum_required(VERSION 3.16)

project(lefdef-SPRoute
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

######################################
# detect the installation path of LEF parser and DEF parser,
# and check if the liblef and libdef library can be found or not
######################################
message(STATUS "Detecting environment variable LEF_ROOT...")
if (DEFINED ENV{LEF_ROOT})
    message(STATUS "Environment variable LEF_ROOT detected: " $ENV{LEF_ROOT})
else ()
    message(FATAL_ERROR "Environment variable LEF_ROOT not found")
endif ()
include_directories($ENV{LEF_ROOT}/include)
link_directories($ENV{LEF_ROOT}/lib)
find_library(LEF_LIBRARY
    NAMES lef
    PATHS $ENV{LEF_ROOT}/lib
    REQUIRED)
if (NOT LEF_LIBRARY)
    message(FATAL_ERROR "liblef.a is not found")
else ()
    message(STATUS "liblef.a found")
endif()

message(STATUS "Detecting environment variable DEF_ROOT...")
if (DEFINED ENV{DEF_ROOT})
    message(STATUS "Environment variable DEF_ROOT detected: " $ENV{DEF_ROOT})
else ()
    message(FATAL_ERROR "Environment variable DEF_ROOT not found")
endif ()
include_directories($ENV{DEF_ROOT}/include)
link_directories($ENV{DEF_ROOT}/lib)
find_library(DEF_LIBRARY
    NAMES def
    PATHS $ENV{DEF_ROOT}/lib
    REQUIRED)
if (NOT DEF_LIBRARY)
    message(FATAL_ERROR "libdef.a is not found")
else ()
    message(STATUS "libdef.a found")
endif()


if(GALOIS_ENABLE_VTUNE)
  set(VTune_ROOT /opt/intel/vtune_amplifier)
  find_package(VTune REQUIRED)
  include_directories(${VTune_INCLUDE_DIRS})
  add_definitions(-DGALOIS_ENABLE_VTUNE)

    set(VTune_LIBRARY_PATH_CANDIDATES lib lib64 lib32 bin64/k1om bin32/k1om)
    find_path(VTune_INCLUDE_DIRS ittnotify.h PATHS ${VTune_ROOT} PATH_SUFFIXES include)
    find_library(VTune_LIBRARY NAMES ittnotify PATHS ${VTune_ROOT} PATH_SUFFIXES ${VTune_LIBRARY_PATH_CANDIDATES})
    find_library(VTune_LIBRARIES NAMES dl PATH_SUFFIXES lib lib64 lib32)
    link_directories(/opt/intel/oneapi/vtune/latest/lib64/)
endif()


message(STATUS "Detecting environment variable $ENV{ACT_HOME}...")
if (DEFINED ENV{ACT_HOME})
    message(STATUS "Environment variable ACT_HOME detected: " $ENV{ACT_HOME})
else ()
    message(FATAL_ERROR "Environment variable $ENV{ACT_HOME} not found")
endif ()
include_directories($ENV{ACT_HOME}/include)
link_directories($ENV{ACT_HOME}/lib)
find_library(PHYDB_LIBRARY
    NAMES phydb
    PATHS $ENV{ACT_HOME}/lib
    REQUIRED)
if (NOT PHYDB_LIBRARY)
    message(FATAL_ERROR "libphydb.a is not found")
else ()
    message(STATUS "libphydb.a found")
endif()

find_library(GALOIS_SHMEM
    NAMES galois_shmem
    PATHS $ENV{ACT_HOME}/lib
    REQUIRED)
if (NOT GALOIS_SHMEM)
    message(FATAL_ERROR "libgalois_shmem.a is not found")
else ()
    message(STATUS "libgalois_shmem.a found")
endif()

find_package(Boost 1.69.0 REQUIRED)
message(STATUS "Boost library path: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost include path: ${Boost_INCLUDE_DIR}")
message(STATUS "Boost libs: ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})

# Set a default build type if none was specified
set(default_build_type "RELEASE")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "DEBUG" "RELEASE")
endif ()

#add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic)
#set(CMAKE_CXX_FLAGS_DEBUG "-g -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-function -Wno-shadow -Wno-unused-parameter")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-function -Wno-shadow -Wno-unused-parameter")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-function -Wno-shadow -Wno-unused-parameter")

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set the output directory of static libraries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Set the output directory of executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include_directories(include)
FILE(GLOB CPP_SOURCES "src/*.cpp")
FILE(GLOB C_SOURCES "src/*.c")
add_subdirectory(test)

add_library(sproute 
    STATIC
    ${CPP_SOURCES} ${C_SOURCES})
target_link_libraries(sproute 
    lef def galois_shmem pthread numa phydb)

add_executable(lefdef-sproute
    test/main.cpp)
target_link_libraries(lefdef-sproute
    lef def galois_shmem sproute pthread numa phydb)

######################################
# specify the installation directory: ${ACT_HOME}
######################################

message(STATUS "Detecting environment variable ACT_HOME...")
if (DEFINED ENV{ACT_HOME})
    message(STATUS "Environment variable ACT_HOME detected: " $ENV{ACT_HOME})
else ()
    message(FATAL_ERROR "Environment variable ACT_HOME not found")
endif ()
message(STATUS "Changing Installation directory to ACT_HOME")
set(CMAKE_INSTALL_PREFIX $ENV{ACT_HOME} CACHE PATH "installation path" FORCE)
message(STATUS "Current installation directory: " ${CMAKE_INSTALL_PREFIX})


######################################
# Install header files
######################################
install(DIRECTORY include/ DESTINATION include/sproute COMPONENT Development FILES_MATCHING PATTERN "*.h")

######################################
# Install library
######################################
install(TARGETS sproute
    DESTINATION lib)
