
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libinverse_kinematics-<target>.so
# and installs in the directory lib/orocos/inverse_kinematics/
#
orocos_component(inverse_kinematics inverse_kinematics-component.hpp inverse_kinematics-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/inverse_kinematics/ directory
#
orocos_install_headers(inverse_kinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libinverse_kinematics-plugin-<target>.so
# and installs in the directory lib/orocos/inverse_kinematics/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(inverse_kinematics-plugin inverse_kinematics-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libinverse_kinematics-service-<target>.so
# and installs in the directory lib/orocos/inverse_kinematics/plugins/
#
orocos_service(inverse_kinematics-service inverse_kinematics-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libinverse_kinematics-support-<target>.so and installs it in
# lib/
#
orocos_library(inverse_kinematics-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.

target_link_libraries(inverse_kinematics
    ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
    )

