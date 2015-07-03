#ifndef OROCOS_LWR_ORO_BRIDGE_COMPONENT_HPP
#define OROCOS_LWR_ORO_BRIDGE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>



#include <lwr_oro_bridge/friComm.h>
#include <FRILibrary/FastResearchInterface.h>
#include <FRILibrary/LinuxAbstraction.h>

#define PI	3.1415926535897932384626433832795
#define NUMBER_OF_JOINTS 7

using namespace std;
using namespace RTT;

class Lwr_oro_bridge : public RTT::TaskContext{
  public:
 
   Lwr_oro_bridge(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();


										/* example function */
										double add(double, double);
										/* reverse param function 
										vector < double >  reverse (vector < double > a);
									    	*/

   private:

	FastResearchInterface *FRI;


		// Internal function declarations


	vector < double > newHometransform( vector < double >  tfmtx, vector < double >  home );

	vector < double > kukaHometransform( vector < double >  cmtfmtx, vector < double >  home );

    	vector < double >  kdlreverseRPY (vector < double > tfmtx);
	
	vector < double >  kdldirectRPY (vector < double > frmvec);

	vector < double >  kdlEEbasewrenchtf (vector < double > eewr, vector < double > tfmtx);


	vector < double >  reverse (vector < double > a);



		// Internal auxiliary variables

	float 		floatValuesMJT[FRI_USER_SIZE], 
	      		floatValuesMEF[6], 
	      		floatValuesMJP[FRI_USER_SIZE],
		        floatValuesMCP[12];


	int	 ResultValue;
	uint16_t fri_state_last;
   	float JointValuesInRad[NUMBER_OF_JOINTS];
	float Transformatrix[12], Stiff[6], Damp[6];	
	float JointStiff[7], JointDamp[7];	

	unsigned int controlScheme;



	int robotStarted;
	bool safetyEnabled; 
	const char* statepointer;   
    
			/*kdl*/
	
	                            	  	

		// Internal vectors	


	vector < double > EEframe;
	vector < double > measCartPos;
	vector < double > initCartPos;

	vector < double > FTwrtEE;
	vector < double > estimExternalCartForces;
	vector < double > measJointPos;
	vector < double > desJointPos;	
	vector < double > measJointTorques;
	
	vector < double > CommCartPos;
	vector < double > CommCartStiff;
	vector < double > CommCartDamp;

	vector < double > MatrixPos;

	vector < double > NewHome;

	vector < double > KukaHome_EEmeastfmtx;
	vector < double > NewHome_EEcmdtfmtx;




		// Input ports	
	

	InputPort < vector < double > > in_desJointPos;
	InputPort < vector < double > > in_CommCartPos;
	InputPort < vector < double > > in_CommCartStiff;	
	InputPort < vector < double > > in_CommCartDamp;	

	
	InputPort < unsigned int > in_controlScheme;
	

		
		// Output ports	
	
  
	OutputPort < vector < double > > out_measJointPos;
	OutputPort < vector < double > > out_measCartPos;	
	OutputPort < vector < double > > out_EEframe;
	OutputPort < vector < double > > out_measJointTorques;
	OutputPort < vector < double > > out_estimExternalCartForces;



	    
	OutputPort < bool > out_safetyEnabled; 
	OutputPort < string > out_FRIstate;
	OutputPort < int > out_robotStarted;


};
#endif




