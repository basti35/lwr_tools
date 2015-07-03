#include "lwr_interface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Lwr_interface::Lwr_interface(std::string const& name) : TaskContext(name){

	this->ports()->addPort( "measJointPos", in_measJointPos ).doc("Measured Joint Positions");
	this->ports()->addPort( "measJointTorques", in_measJointTorques ).doc("Measured Joint Torques");
	this->ports()->addPort( "FRIstate", in_FRIstate).doc("State of the FRI (MON/COM/OFF)");

  std::cout << "Lwr_interface constructed !" <<std::endl;
}

bool Lwr_interface::configureHook(){

	measJointPos.resize(7);
	measJointTorques.resize(7);

  std::cout << "Lwr_interface configured !" <<std::endl;
  return true;
}

bool Lwr_interface::startHook(){

	fri_state = "unknown_mode";


  std::cout << "Lwr_interface started !" <<std::endl;
  return true;
}

void Lwr_interface::updateHook(){

	if (in_FRIstate.read(fri_state) == NewData)
	{
		if (fri_state.compare("off") == 0)
			cout << "No UDP connection has been established" << endl;
		if (fri_state.compare("monitor_mode") == 0)
			cout << "Switching to monitor mode" << endl;
		if (fri_state.compare("command_mode") == 0)
			cout << "Switching to command mode" << endl;
		if (fri_state.compare("unknown_mode") == 0)
			cout << "Unknown mode" << endl;
	}
	
	in_measJointPos.read(measJointPos);
	in_measJointTorques.read(measJointTorques);

	/* Measured joint positions [rad] */
	for (unsigned int i = 0; i < 7; i++)
	{
		printf("Joint position %d: %8.3f degrees\n", i, measJointPos[i] * 180.0 / PI);
	}
	printf("\n\n");

	/* Measured joint torques [Nm] */
	// for (unsigned int i = 0; i < 7; i++)
	// {
	// 	printf("Joint torque %d: %8.3f Nm\n", i, measJointTorques[i]);
	// }
	// printf("\n\n");

  //std::cout << "Lwr_interface executes updateHook !" <<std::endl;
}

void Lwr_interface::stopHook() {
  std::cout << "Lwr_interface executes stopping !" <<std::endl;
}

void Lwr_interface::cleanupHook() {
  std::cout << "Lwr_interface cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Lwr_interface)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Lwr_interface)
