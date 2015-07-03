
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library liblwr_oro_bridge-<target>.so
# and installs in the directory lib/orocos/lwr_oro_bridge/
#
orocos_component(lwr_oro_bridge lwr_oro_bridge-component.hpp lwr_oro_bridge-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

target_link_libraries (lwr_oro_bridge FastResearchInterfaceLibrary.a)
target_link_libraries (lwr_oro_bridge rt)

#
# Additional headers:
#
# Installs in the include/orocos/lwr_oro_bridge/ directory
#
orocos_install_headers(lwr_oro_bridge-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library liblwr_oro_bridge-plugin-<target>.so
# and installs in the directory lib/orocos/lwr_oro_bridge/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(lwr_oro_bridge-plugin lwr_oro_bridge-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library liblwr_oro_bridge-service-<target>.so
# and installs in the directory lib/orocos/lwr_oro_bridge/plugins/
#
orocos_service(lwr_oro_bridge-service lwr_oro_bridge-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library liblwr_oro_bridge-types-<target>.so
# and installs in the directory lib/orocos/@target@/lwr_oro_bridge/types/
#
# The header will go in include/orocos/lwr_oro_bridge/types/lwr_oro_bridge/lwr_oro_bridge-types.hpp
# So you can #include <lwr_oro_bridge/lwr_oro_bridge-types.hpp>
#
orocos_typegen_headers(include/lwr_oro_bridge/lwr_oro_bridge-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


