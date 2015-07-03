#ifndef OROCOS_LWR_INTERFACE_COMPONENT_HPP
#define OROCOS_LWR_INTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <stdio.h>
#include <stdlib.h>

#define PI	3.1415926535897932384626433832795

using namespace std;
using namespace RTT;

class Lwr_interface : public RTT::TaskContext{
  public:
    Lwr_interface(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    private:

	vector < double > measJointPos;
    vector < double > measJointTorques;
    string fri_state;

	InputPort < vector< double > > in_measJointPos;
    InputPort < vector< double > > in_measJointTorques;
    InputPort < string > in_FRIstate;

};
#endif
