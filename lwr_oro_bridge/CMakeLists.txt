#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(lwr_oro_bridge)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/lwr_oro_bridge/friComm.h)
  message(FATAL_ERROR "
   You NEED TO copy the friComm.h file that you got from KUKA in this directory:
   ${CMAKE_CURRENT_SOURCE_DIR}/include/lwr_oro_bridge/

   Otherwise, this will never compile !
   ")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(orocos_kdl)
include_directories(${KDL_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(/home/kuka/KUKA/FRILibrary/include/)
link_directories(/home/kuka/KUKA/FRILibrary/Linux/x64/release/lib)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library liblwer-types-<target>.so
# and installs in the directory lib/orocos/@target@/lwer/types/
#
# The header will go in include/orocos/lwer/types/lwer/lwer-types.hpp
# So you can #include <lwer/lwer-types.hpp>
#
#orocos_typegen_headers(include/lwr_oro_bridge/lwr_oro_bridge-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
