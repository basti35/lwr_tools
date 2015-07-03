
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library liblwr_interface-<target>.so
# and installs in the directory lib/orocos/lwr_interface/
#
orocos_component(lwr_interface lwr_interface-component.hpp lwr_interface-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/lwr_interface/ directory
#
orocos_install_headers(lwr_interface-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library liblwr_interface-plugin-<target>.so
# and installs in the directory lib/orocos/lwr_interface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(lwr_interface-plugin lwr_interface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library liblwr_interface-service-<target>.so
# and installs in the directory lib/orocos/lwr_interface/plugins/
#
orocos_service(lwr_interface-service lwr_interface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library liblwr_interface-types-<target>.so
# and installs in the directory lib/orocos/@target@/lwr_interface/types/
#
# The header will go in include/orocos/lwr_interface/types/lwr_interface/lwr_interface-types.hpp
# So you can #include <lwr_interface/lwr_interface-types.hpp>
#
orocos_typegen_headers(include/lwr_interface/lwr_interface-types.hpp) # ...you may add multiple header files
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


