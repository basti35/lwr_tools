#include "lwr_oro_bridge-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>
#include <sstream>

#define K 2000
#define D 0.7
#define EE_DOFS 6

Lwr_oro_bridge::Lwr_oro_bridge(std::string const& name) : TaskContext(name){


/* Input ports */

	this->ports()->addPort( "desJointPos", in_desJointPos ).doc("Desired Joint Positions");
	
	this->ports()->addPort( "CommCartPos", in_CommCartPos ).doc("Commanded cartesian frame");
	this->ports()->addPort( "CommCartStiff", in_CommCartStiff ).doc("Commanded cartesian stiff");
	this->ports()->addPort( "CommCartDamp", in_CommCartDamp ).doc("Commanded cartesian damping");
	this->ports()->addPort( "controlScheme", in_controlScheme ).doc("Control strategy to be applied");




/* Output ports */

	this->ports()->addPort( "measJointPos", out_measJointPos ).doc("Measured Joint Positions");
	this->ports()->addPort( "measCartPos", out_measCartPos ).doc("Measured Transformation matrix w.r.t EE");
	this->ports()->addPort( "measJointTorques", out_measJointTorques ).doc("Measured Joint Torques");
	this->ports()->addPort( "estimExternalCartForces", out_estimExternalCartForces).doc("Measured external cartesian forces");
	this->ports()->addPort( "EEframe", out_EEframe).doc("Measured xyzRPY frame of EE");


	this->ports()->addPort( "FRIstate", out_FRIstate).doc("State of the FRI (MON/COM/OFF)");
	this->ports()->addPort( "safetyEnabled", out_safetyEnabled ).doc("Safety Enabled");
	this->ports()->addPort( "robotStarted", out_robotStarted).doc("Robot has been started");



std::cout << "Lwr_oro_bridge constructed !" <<std::endl;

}





bool Lwr_oro_bridge::configureHook(){

	FRI = new FastResearchInterface("/home/kuka/KUKA/FRILibrary/etc/980039-FRI-Driver.init");


	measJointPos.resize(LBR_MNJ);
	desJointPos.resize(LBR_MNJ);
	measJointTorques.resize(LBR_MNJ);
	estimExternalCartForces.resize(6);
	measCartPos.resize(12);
	MatrixPos.resize(12);
	EEframe.resize(6);
	
	FTwrtEE.resize(6);

	NewHome.resize(12);	
	initCartPos.resize(12);	
		

	CommCartPos.resize(6);
	CommCartStiff.resize(6);
	CommCartDamp.resize(6);

	
	out_EEframe.setDataSample(EEframe);
	out_estimExternalCartForces.setDataSample(estimExternalCartForces);


	KukaHome_EEmeastfmtx.resize(12);				///////////////////////////////////////////
	NewHome_EEcmdtfmtx.resize(12);



	//controlScheme = FastResearchInterface::CART_IMPEDANCE_CONTROL;
	//controlScheme = FastResearchInterface::JOINT_POSITION_CONTROL;			///////////////////////////////
	ResultValue = 0;
	robotStarted = 0;


  std::cout << "Lwr_oro_bridge configured !" <<std::endl;
  return true;
}





bool Lwr_oro_bridge::startHook(){

cout << endl << "Please select the Control Mode on both the computer and the KCP: " << endl << endl;
	
	cout << "1 -----> Position control" << endl;
	cout << "2 -----> Cartesian impedance control" << endl;
	cout << "3 -----> Joint impedance control" << endl;
	cout << "4 -----> Joint torque control" << endl;
	
cout << endl << "PS: first time better start up with position control and then switching to stiffness restarting the component ! " << endl;
	cout << endl;

	unsigned int in;

retry:

string mystr;

getline(cin,mystr);

stringstream(mystr) >> in;



   switch (in)
	{
		case 1:
		controlScheme = FastResearchInterface::JOINT_POSITION_CONTROL;		
		break;

		case 2:
		controlScheme = FastResearchInterface::CART_IMPEDANCE_CONTROL;		
		break;

		case 3:
		controlScheme = FastResearchInterface::JOINT_IMPEDANCE_CONTROL;		
		break;

		case 4:
		controlScheme = FastResearchInterface::JOINT_TORQUE_CONTROL;		
		break;
		
		default:  cout << "Control mode not recognized... try again..." << endl;
			 goto retry;
	}





cout << "CHECK OUT if ethernet is properly connected !" << endl;

cout << "Please start up the robot now by using KUKA Control Panel WITH THE SELECTED CONTROL MODE !" << endl;
	ResultValue	= FRI->StartRobot(controlScheme, 120.0);

	if (ResultValue != EOK)
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	}
	

	
	/*********** INITIALIZE DATA ************/
	
	

	/* Measured joint torques [Nm] */
	FRI->GetMeasuredJointTorques(floatValuesMJT);
	for (unsigned int i = 0; i < LBR_MNJ; i++)
	{

		measJointTorques[i] = floatValuesMJT[i];
	}
		
	/* Measured joint positions [rad] */
	FRI->GetMeasuredJointPositions(floatValuesMJP);
	for (unsigned int i = 0; i < LBR_MNJ; i++)
	{
		measJointPos[i] = floatValuesMJP[i];
	}



	/*Measured TRANSFORMATION MATRIX and REVERSING TO GET POSE*/
	FRI->GetMeasuredCartPose(floatValuesMCP);
	
	
	for (unsigned int i = 0; i < 12; i++)
	{

	
		NewHome[i] = KukaHome_EEmeastfmtx[i] = (double)floatValuesMCP[i];
	}
	
		initCartPos = measCartPos = newHometransform(KukaHome_EEmeastfmtx, NewHome);


	
	/* Measured External Forces on E.E. [N,Nm] */
	FRI->GetEstimatedExternalCartForcesAndTorques(floatValuesMEF);
	for (unsigned int i = 0; i < 6; i++)
	{
		FTwrtEE[i] = floatValuesMEF[i];					// Reads forces and torques wrt E.E. frame
	}

		
	estimExternalCartForces =  kdlEEbasewrenchtf(FTwrtEE, measCartPos);     // Trabsforms wrt the Base frame		


			
	/* Initializing desired joint positions */

	for (unsigned int i = 0; i < LBR_MNJ; i++)
	{
		desJointPos[i] = measJointPos[i];
	}							

	for (unsigned int i=0; i < 12; i++)		



////////////////////////////////// INITIALIZING CARTESIAN STIFFNESS CONTROL //////////////////////////////////////////
		
	{
	   Transformatrix[i] = measCartPos[i]; 
	}


	CommCartPos = kdlreverseRPY(measCartPos);

	EEframe =kdlreverseRPY(measCartPos);


	Stiff[0] = K;
	Stiff[1] = K;
	Stiff[2] = K;
	Stiff[3] = K;
	Stiff[4] = K;
	Stiff[5] = K;



	Damp[0] = D;
	Damp[1] = D;
	Damp[2] = D;
	Damp[3] = D;
	Damp[4] = D;
	Damp[5] = D;

	
	safetyEnabled = FRI->IsMachineOK();	

	 



	out_EEframe.write(EEframe);		
	out_measCartPos.write(measCartPos);	
	out_measJointPos.write(measJointPos);
	out_measJointTorques.write(measJointTorques);
	out_estimExternalCartForces.write(estimExternalCartForces);




	statepointer = FRI -> GetCompleteRobotStateAndInformation();
	

	cout << endl << endl << "Courrent complete state of the robot:" << ' ' << statepointer << endl << endl;

	/*********************************/

										//Example function

										double d=2,p=3;

										double h=add(d,p);

										cout<<"funzione esempio dà: "<< h << endl;
	




  std::cout << "Lwr_oro_bridge started !" <<std::endl;
  return true;
}

void Lwr_oro_bridge::updateHook(){



	/*********** FRI STATE ***********/
	/* Check if the state of the FRI is changed */
	if (FRI->GetFRIMode() == FRI_STATE_OFF) 
	{
           if (fri_state_last != FRI->GetFRIMode())
	        {
			out_FRIstate.write("off");
			out_FRIstate.write("monitor_mode");
		}
	} 
		else if (FRI->GetFRIMode() == FRI_STATE_MON) 
		{
		    if (fri_state_last != FRI->GetFRIMode()) 
			{}
		} 
	  		else if (FRI->GetFRIMode() == FRI_STATE_CMD && FRI->IsRobotArmPowerOn()) 
			    {
				if (fri_state_last != FRI->GetFRIMode()) 
				   {
					out_FRIstate.write("command_mode");
				    }
		             } 
			        else 
				      {
			if (fri_state_last != FRI->GetFRIMode())
				out_FRIstate.write("unknown_mode");
				      }
	fri_state_last = FRI->GetFRIMode();
	/*********************************/


	/******* CONTROL STRATEGY ********/

	
	 if ( in_controlScheme.read(controlScheme) == NewData)
	{

		
	statepointer = FRI -> GetCompleteRobotStateAndInformation();
	
	cout << endl << endl << "Courrent complete state of the robot:" << ' ' << statepointer << endl << endl;



		cout << "Program is going to stop the robot" << endl;
		ResultValue = FRI->StopRobot();
		robotStarted = 0;
		if (ResultValue != EOK)
			{
			fprintf(stderr, "ERROR, could not stop robot: %s\n", strerror(ResultValue));
			}

		cout << "Restarting using the selected controller" << endl;
		ResultValue	= FRI->StartRobot(controlScheme, 120.0);
		if (ResultValue != EOK)
		{
			fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
		}
		else    {
			robotStarted = 1;
			cout << "Robot started" << endl;
			}

	}	

	

	out_robotStarted.write(robotStarted);


	FRI->WaitForKRCTick();


	statepointer = FRI -> GetCompleteRobotStateAndInformation();

	static int ctr;

  if (!FRI->IsMachineOK())
	{
  /*  if(ctr < 3)
		{	
			cout << "ERROR, safety circuit unactivated. ctr: " << ctr <<endl ;
			cout << endl << endl << "Courrent complete state of the robot:" << ' ' << statepointer << endl << endl;
			ctr++;
		}
*/	}
	else
		ctr=0;






	switch( controlScheme ){


//-------------------------------------------------------------------------------------------------------------------------------


				/* JOINT POSITION CONTROL */				


		case FastResearchInterface::JOINT_POSITION_CONTROL: 
			
	in_desJointPos.read(desJointPos);

	
		

				for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++){
					JointValuesInRad[i]	=	(float)desJointPos[i];
				}


				//-setting control input-//

	FRI->SetCommandedJointPositions(JointValuesInRad);	

		break;

//--------------------------------------------------------------------------------------------------------------------------------


 			       /* CARTESIAN STIFFNESS CONTROL */


	        case FastResearchInterface::CART_IMPEDANCE_CONTROL:



		in_CommCartPos.read(CommCartPos);


		MatrixPos = kdldirectRPY(CommCartPos);	// To find the transformation matrix with direct analysis from POSE


		NewHome_EEcmdtfmtx = kukaHometransform (MatrixPos, NewHome);// To find the tf mtx. in the original kuka ref. sys.
	

		for (unsigned int i = 0; i < 12; i++)
			{
			Transformatrix[i]	=	(float)NewHome_EEcmdtfmtx[i];
			}



	if (in_CommCartStiff.read(CommCartStiff) == NewData)
	{

		in_CommCartStiff.read(CommCartStiff);
	
		for (unsigned int i = 0; i < EE_DOFS; i++)
			{
			Stiff[i]  =  (float)CommCartStiff[i];
			}
	}	


	if (in_CommCartDamp.read(CommCartDamp) == NewData)
 
	{
	
		in_CommCartDamp.read(CommCartDamp);

		for (unsigned int i = 0; i < EE_DOFS; i++)
			{
			Damp[i]	=	(float)CommCartDamp[i];
			}
	}


				//-setting control inputs-//


	FRI -> SetCommandedCartStiffness (Stiff);
	FRI -> SetCommandedCartDamping (Damp);
	FRI -> SetCommandedCartPose (Transformatrix);



		  	break;

//--------------------------------------------------------------------------------------------------------------------------------




		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			

		for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++)
			{
			JointValuesInRad[i]	=	(float)desJointPos[i];
			}


				//-setting control input-//

	FRI -> SetCommandedJointPositions (JointValuesInRad);	


		for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++)
			{
			JointStiff[i]	=	(float)K;
			}


	FRI -> SetCommandedJointStiffness (JointStiff);



		for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++)
			{
			JointDamp[i]	=	(float)D;
			}


	FRI -> SetCommandedJointStiffness (JointDamp);
		
			break;


//--------------------------------------------------------------------------------------------------------------------------------




		case FastResearchInterface::JOINT_TORQUE_CONTROL:
			/* Joint impedance control */
			// TODO
			break;


//--------------------------------------------------------------------------------------------------------------------------------
	
	}





	
	/*********** UPDATE DATA ************/
	
	/* Measured joint torques [Nm] */
	FRI->GetMeasuredJointTorques(floatValuesMJT);
	for (unsigned int i = 0; i < LBR_MNJ; i++)
	{
		measJointTorques[i] = floatValuesMJT[i];
	}
	

	/* Measured joint positions [rad] */
	FRI->GetMeasuredJointPositions(floatValuesMJP);
	for (unsigned int i = 0; i < LBR_MNJ; i++)
	{
		measJointPos[i] = floatValuesMJP[i];
	}


	
	/* Measured TRANSFORMATION MATRIX and REVERSING TO GET POSE*/
	FRI->GetMeasuredCartPose(floatValuesMCP);
	

	for (unsigned int i = 0; i < 12; i++)
	{	

		KukaHome_EEmeastfmtx[i] = (double)floatValuesMCP[i];
	}
	
		initCartPos = measCartPos = newHometransform(KukaHome_EEmeastfmtx, NewHome);

	
	/* Measured External Forces on E.E. [N,Nm] */
	FRI->GetEstimatedExternalCartForcesAndTorques(floatValuesMEF);
	for (unsigned int i = 0; i < 6; i++)
	{
		FTwrtEE[i] = floatValuesMEF[i];					// Reads forces and torques wrt E.E. frame
	}

		
		estimExternalCartForces =  kdlEEbasewrenchtf(FTwrtEE,measCartPos);     // Trabsforms wrt the Base frame		

	


	/* Updating Robot Condition */

	safetyEnabled = FRI->IsMachineOK() ;
	


	EEframe = kdlreverseRPY(measCartPos); 


	out_EEframe.write(EEframe);			
	out_safetyEnabled.write(safetyEnabled);
	out_measCartPos.write(measCartPos);	
	out_measJointPos.write(measJointPos);
	out_measJointTorques.write(measJointTorques);
	out_estimExternalCartForces.write(estimExternalCartForces);

	/*********************************/


}





void Lwr_oro_bridge::stopHook() {


	statepointer = FRI -> GetCompleteRobotStateAndInformation();
	

	cout << endl << endl << "Courrent complete state of the robot:" << ' ' << statepointer << endl << endl;


std::cout << "Waiting from KRC input" <<std::endl;

	
	ResultValue = 	FRI->StopRobot();


	if (ResultValue != EOK)

	{

		fprintf(stderr, "ERROR, could not stop robot: %s\n", strerror(ResultValue));
	}


  std::cout << "Lwr_oro_bridge executes stopping !" <<std::endl;
}







void Lwr_oro_bridge::cleanupHook() {

cout << "Deleting the object FRI" << endl;
	
	delete FRI;

  std::cout << "Lwr_oro_bridge cleaning up !" <<std::endl;
}





















/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//.................................................Function Definitions........................................................//






//KDL Providing transformatrix with respect to the new selected Home


	vector < double > Lwr_oro_bridge::newHometransform( vector < double >  tfmtx, vector < double >  home )

	{	
	vector < double > result;

		tfmtx.resize(12);
		home.resize(12);
		result.resize(12);

		KDL::Vector P_PAn(tfmtx[3],tfmtx[7],tfmtx[11]);
		KDL::Rotation R_PAn(tfmtx[0],tfmtx[1],tfmtx[2],tfmtx[4],tfmtx[5],tfmtx[6],tfmtx[8],tfmtx[9],tfmtx[10]);
		KDL::Frame F_PAn(R_PAn,P_PAn);			// Point tfmtx wrt original kuka base frame

		KDL::Vector P_BAn(home[3],home[7],home[11]);
		KDL::Rotation R_BAn(home[0],home[1],home[2],home[4],home[5],home[6],home[8],home[9],home[10]);
		KDL::Frame F_BAn(R_BAn,P_BAn);				// New home tfmtx wrt original kuka base frame


		KDL::Frame invF_BAn;	
		invF_BAn = F_BAn.Inverse();

		KDL::Frame F_PBn;

		F_PBn = invF_BAn*F_PAn ;			// New home tfmtx wrt original kuka base frame in base tern


		result[0] = F_PBn(0,0);	
		result[1] = F_PBn(0,1);	
		result[2] = F_PBn(0,2);	
		result[3] = F_PBn(0,3);	

		result[4] = F_PBn(1,0);	
		result[5] = F_PBn(1,1);	
		result[6] = F_PBn(1,2);	
		result[7] = F_PBn(1,3);	

		result[8] = F_PBn(2,0);	
		result[9] = F_PBn(2,1);	
		result[10] = F_PBn(2,2);	
		result[11] = F_PBn(2,3);	

		

		return (result);
	}	






//KDL Providing transformatrix with respect to the Kuka original Home


	vector < double > Lwr_oro_bridge::kukaHometransform( vector < double >  cmtfmtx, vector < double >  home )

	{	
	vector < double > result;

		cmtfmtx.resize(12);

		home.resize(12);
		result.resize(12);

		KDL::Vector P_PBo(cmtfmtx[3],cmtfmtx[7],cmtfmtx[11]);
		KDL::Rotation R_PBo(cmtfmtx[0],cmtfmtx[1],cmtfmtx[2],cmtfmtx[4],cmtfmtx[5],cmtfmtx[6],cmtfmtx[8],cmtfmtx[9],cmtfmtx[10]);
		KDL::Frame F_PBo(R_PBo,P_PBo);			// Point tfmtx wrt original kuka base frame

		KDL::Vector P_BAo(home[3],home[7],home[11]);
		KDL::Rotation R_BAo(home[0],home[1],home[2],home[4],home[5],home[6],home[8],home[9],home[10]);
		KDL::Frame F_BAo(R_BAo,P_BAo);				

		KDL::Frame F_PAo;

		F_PAo = F_BAo * F_PBo ;				// New home tfmtx wrt original kuka base frame in base tern


		result[0] = F_PAo(0,0);	
		result[1] = F_PAo(0,1);	
		result[2] = F_PAo(0,2);	
		result[3] = F_PAo(0,3);	

		result[4] = F_PAo(1,0);	
		result[5] = F_PAo(1,1);	
		result[6] = F_PAo(1,2);	
		result[7] = F_PAo(1,3);	

		result[8] = F_PAo(2,0);	
		result[9] = F_PAo(2,1);	
		result[10] = F_PAo(2,2);	
		result[11] = F_PAo(2,3);	

		

		return (result);
	}	






//KDL reverse parametrization RPY function


	vector < double > Lwr_oro_bridge::kdlreverseRPY( vector < double >  tfmtx)

	{	
	vector < double > result;

		tfmtx.resize(12);
		result.resize(6);

		KDL::Vector p(tfmtx[3],tfmtx[7],tfmtx[11]);

		KDL::Rotation M(tfmtx[0],tfmtx[1],tfmtx[2],tfmtx[4],tfmtx[5],tfmtx[6],tfmtx[8],tfmtx[9],tfmtx[10]);
	
		KDL::Frame f4(M,p);


			result[0] = tfmtx[3];	// x coord.

			result[1] = tfmtx[7];	// y coord.

			result[2] = tfmtx[11];	// z coord.

	
		f4.M.GetRPY(result[3],result[4],result[5]);

		result[3] = result[3] ;
		
		result[4] = result[4] ;

		result[5] = result[5] ;


		return (result);
	}	






//KDL direct parametrization RPY function


	vector < double > Lwr_oro_bridge::kdldirectRPY( vector < double >  frmvec)

	{	
	vector < double > result;

		frmvec.resize(6);
		result.resize(12);

		KDL::Vector p(frmvec[0],frmvec[1],frmvec[2]);

		

		KDL::Rotation M = KDL::Rotation::RPY(frmvec[3],frmvec[4],frmvec[5]);
	
		KDL::Frame f4(M,p);


		result[0] = f4(0,0);	
		result[1] = f4(0,1);	
		result[2] = f4(0,2);	
		result[3] = f4(0,3);	

		result[4] = f4(1,0);	
		result[5] = f4(1,1);	
		result[6] = f4(1,2);	
		result[7] = f4(1,3);	

		result[8] = f4(2,0);	
		result[9] = f4(2,1);	
		result[10] = f4(2,2);	
		result[11] = f4(2,3);	

	

		return (result);
	}	






//KDL wrench EE-Base reference transformation 


	vector < double > Lwr_oro_bridge::kdlEEbasewrenchtf( vector < double >  eewr, vector < double >  tfmtx)

	{	
	vector < double > result;

		eewr.resize(6);
		tfmtx.resize(12);

		result.resize(6);

//		KDL::Vector p(a[3],a[7],a[11]);

		KDL::Rotation M(tfmtx[0],tfmtx[1],tfmtx[2],tfmtx[4],tfmtx[5],tfmtx[6],tfmtx[8],tfmtx[9],tfmtx[10]);
	
/*		KDL::Frame f4(M,p);


			result[0] = a[3];	// x coord.

			result[1] = a[7];	// y coord.

			result[2] = a[11];	// z coord.
*/
		KDL::Vector force(eewr[0], eewr[1], eewr[2]);
		KDL::Vector torque(eewr[3], eewr[4], eewr[5]);

		KDL::Wrench Wee(force,torque);		
		KDL::Wrench Wbase;		
		
		Wbase = M*Wee;

		result[0] = Wbase(0);
		result[1] = Wbase(1);
		result[2] = Wbase(2);
		result[3] = Wbase[3];
		result[4] = Wbase[4];
		result[5] = Wbase[5];
		


		return (result);
	}	







/* MY reverse parametrization function*/


							vector < double > Lwr_oro_bridge::reverse( vector < double >  a)	
							{

								a.resize(12);


							double g,t,f ;
							vector < double > result;

	
								result.resize(6);


								g= atan2( -a[4], -a[0]);

								t= atan2( -a[8],-sqrt(pow(a[9],2)+pow(a[10],2)));

								f= atan2(-a[9],-a[10]);


								result[0] = a[3];

								result[1] = a[7];

								result[2] = a[11];

								result[3] = f;

								result[4] = t;

								result[5] = g;

								return ( result);

							}

							





// Example function

										double Lwr_oro_bridge::add (double a, double b)

										{
										double c;

										c = a+b;

										return(c);
										}








//...............................................................................................................................



/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Lwr_oro_bridge)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */

ORO_CREATE_COMPONENT(Lwr_oro_bridge)
