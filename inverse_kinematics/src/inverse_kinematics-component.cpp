#include "inverse_kinematics-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace KDL;
using namespace RTT;

Inverse_kinematics::Inverse_kinematics(std::string const& name) : TaskContext(name){

	this->ports()->addPort("o_jnt_pos", o_jnt_pos_).doc( "Send joint position (JntArray)" );
  this->ports()->addPort("o_jnt_pos_double", o_jnt_pos_double_).doc( "Send joint position (double)" );
  this->ports()->addPort("o_fk", o_fk_).doc( "Output of the forward kinematics" );
  this->ports()->addPort("o_cart_pos", o_cart_pos_).doc( "Send cartesian position" );
	this->ports()->addPort("i_cart_pos", i_cart_pos_).doc( "Read the cartesian set point" );
	this->ports()->addEventPort("i_rob_pos", i_rob_pos_).doc( "Read robot position" );

	this->addProperty("q_min", q_min_prop_).doc("the minimum joint positions");
	this->addProperty("q_max", q_max_prop_).doc("the maximum joint positions");
  this->addProperty("weights", weights_prop_).doc("weights for NSO");




  std::cout << "Inverse_kinematics constructed !" <<std::endl;
}

bool Inverse_kinematics::configureHook(){

	// Load KUKA LWR chain
	model_ = loadKukaLWR();


  // Initializing joint limits
  q_min_prop_.resize(model_.getNrOfJoints());
  q_max_prop_.resize(model_.getNrOfJoints());
  q_min_.resize(model_.getNrOfJoints());
  q_max_.resize(model_.getNrOfJoints());
  q_opt_.resize(model_.getNrOfJoints());
  weights_.resize(model_.getNrOfJoints());
  weights_prop_.resize(model_.getNrOfJoints());

  //Inizializing JointArray_init
  q_init_.resize(model_.getNrOfJoints());
  jnt_pos_double_.resize(model_.getNrOfJoints());

  //Inizializing port comunication
  o_jnt_pos_double_.setDataSample(jnt_pos_double_);
  o_jnt_pos_.setDataSample(JntArray(model_.getNrOfJoints()));

  std::cout << "Inverse_kinematics configured !" <<std::endl;
  return true;
}


bool Inverse_kinematics::startHook(){

	// Set joint limits
  for (int i = 0; i < model_.getNrOfJoints(); ++i){
      q_min_(i) = q_min_prop_[i];
      q_max_(i) = q_max_prop_[i];
      q_init_(i) = 0.0;
      q_opt_(i) = (q_max_(i) + q_min_(i)) / 2;
      jnt_pos_double_[i] = 0.0;
      weights_(i) = weights_prop_[i];
  }

  L_(0)=1;L_(1)=1;L_(2)=1;
  L_(3)=0.1;L_(4)=0.1;L_(5)=0.1;

  q_init_(1) = 1.57;
  jnt_pos_double_[1] = 1.57;

	//Initializing solvers:
 	fk_solver_ = new ChainFkSolverPos_recursive(model_);													                             // Forward position solver
  #if USE_PINV_NSO
    ik_solver_vel_ = new ChainIkSolverVel_pinv_nso(model_, q_opt_, weights_);             // DOES NOT WORK!!
  #endif
  #if USE_PINV
    ik_solver_vel_ = new ChainIkSolverVel_pinv(model_); 										                // Inverse velocity solver
  #endif

  #if USE_NR_JL
    iksolver_pos_ = new ChainIkSolverPos_NR_JL(model_, q_min_, q_max_, *fk_solver_, *ik_solver_vel_, 100, 1e-6);     // Inverse position solver   --   100 iterations, accuracy 1e-6
  #endif

  #if USE_NR
    iksolver_pos_ = new ChainIkSolverPos_NR(model_, *fk_solver_, *ik_solver_vel_, 100, 1e-6);
  #endif

  #if USE_LMA
    iksolver_pos_ = new ChainIkSolverPos_LMA(model_, L_, 1e-5, 500, 1e-15);
  #endif

  std::cout << "Inverse_kinematics started !" <<std::endl;
  return true;
}


void Inverse_kinematics::updateHook(){

	// Read the robot position
//  i_rob_pos_.read(q_init_);
  //std::cout << q_init(0) << " " << q_init(1) << " " << q_init(2) << " " << q_init(3) << " " << q_init(4) << " " << q_init(5)  <<std::endl;
 
  // Forward kinematics
  getCartPosition();

  JntArray q;
  q.resize(model_.getNrOfJoints());

  // Read a Frame to be transformed
  if (i_cart_pos_.read(position_) == RTT::NewData){    
    std::cout << "Position: " << position_ << std::endl;

    // Calculate inverse kinematics
    int ret = iksolver_pos_->CartToJnt(q_init_, position_, q);
    //std::cout << jnt_pos_double(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6) <<std::endl;

    // Angle reshape from -M_PI to +M_PI
    for (int i = 0; i < model_.getNrOfJoints(); ++i){
      q(i) = reshapeAngle(q(i));
      jnt_pos_double_[i] = q(i);
     }
     //std::cout << jnt_pos_double[0] << " " << jnt_pos_double[1] << " " << jnt_pos_double[2] << " " << jnt_pos_double[3] << " " << jnt_pos_double[4] << " " << jnt_pos_double[5] << " " << jnt_pos_double[5]  <<std::endl;

    for (int i = 0; i < model_.getNrOfJoints(); ++i){
      q_init_(i) = q(i);
     }


    if(ret < 0){
      std::cout << "Error in CartToJnt " << ret <<std::endl;
    }
    else{
      std::cout << "CartToJnt OK " <<std::endl;
      // Write the joint positions
      o_jnt_pos_.write(q);
      o_jnt_pos_double_.write(jnt_pos_double_);
      std::cout << jnt_pos_double_[0] << " " << jnt_pos_double_[1] << " " << jnt_pos_double_[2] << " " << jnt_pos_double_[3] << " " << jnt_pos_double_[4] << " " << jnt_pos_double_[5] << " " << jnt_pos_double_[5]  <<std::endl;
      //std::cout << this->getName() << q << std::endl;
    }

  }

}


void Inverse_kinematics::stopHook() {


  std::cout << "Inverse_kinematics executes stopping !" <<std::endl;
}


void Inverse_kinematics::cleanupHook() {


  std::cout << "Inverse_kinematics cleaning up !" <<std::endl;
}



Frame Inverse_kinematics::getCartPosition(){

  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(model_);
  
  KDL::Frame cartpos; 
  std::vector<double> fk;
  fk.resize(6);   

  // Read the robot position
  i_rob_pos_.read(q_init_);

  // Calculate forward position kinematics
  bool kinematics_status;

  kinematics_status = fksolver.JntToCart(q_init_, cartpos);
  
  if(kinematics_status >= 0){ 
    for (int i = 0; i < 3; ++i)
      fk[i] = cartpos.p[i];
    cartpos.M.GetRPY(fk[3],fk[4],fk[5]);
    std::cout << fk[0] << " " << fk[1] << " " << fk[2] << " " << fk[3] << " " << fk[4] << " " << fk[5]  <<std::endl;
    o_fk_.write(fk);
    o_cart_pos_.write(cartpos);   
    return cartpos;  
  }
  else{
    std::cout<<"Error in getCartPosition!"<<std::endl;
    for (int i = 0; i < 3; ++i)
      fk[i] = cartpos.p[i];
    cartpos.M.GetRPY(fk[3],fk[4],fk[5]);
    o_fk_.write(fk);
    o_cart_pos_.write(cartpos);
    return cartpos;
  }
  
}



double Inverse_kinematics::reshapeAngle(double angle){

  while (angle > M_PI) 
    angle = angle - 2*M_PI;
  
  while (angle < -M_PI)
    angle = angle + 2*M_PI;
  

return angle;

}



// Function that load the KUKA LWR chain
KDL::Chain Inverse_kinematics::loadKukaLWR(){
  
	KDL::Chain kukaLWR;

	// Joint 0
	kukaLWR.addSegment(Segment(Joint(Joint::None),
                        Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
                        ));

  	// Joint 1
  	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
                        Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                        Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector::Zero(),
																						RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

  	// Joint 2
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector(0.0,-0.3120511,-0.0038871),
																						RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

	// Joint 3
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector(0.0,-0.0015515,0.0),
																						RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

	// Joint 4
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
						Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector(0.0,0.5216809,0.0),
																						RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

	// Joint 5
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
						Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector(0.0,0.0119891,0.0),
																						RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

	// Joint 6
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
						Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
																						Vector(0.0,0.0080787,0.0),
																						RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));

	// Joint 7
	kukaLWR.addSegment(Segment(Joint(Joint::RotZ),
						Frame::Identity(),
						RigidBodyInertia(2,
								Vector::Zero(),
								RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));

  return kukaLWR;

}



//Function that load the puma 260 chain
KDL::Chain Inverse_kinematics::loadPuma260(){
  KDL::Chain p260;

  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.0,M_PI_2,0.0,0.0),
                          RigidBodyInertia()));
  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.203202,0.0,0.0,0.0),
                          RigidBodyInertia()));
  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.0,-M_PI_2,-0.12623,0.0),
                          RigidBodyInertia()));
  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.0,M_PI_2,0.2032,0.0),
                          RigidBodyInertia()));
  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.0,-M_PI_2,0.0,0.0),
                          RigidBodyInertia()));
  p260.addSegment(Segment(Joint(Joint::RotZ),
                          Frame::DH(0.0,0.0,0.05537,0.0),
                          RigidBodyInertia()));

  return p260;

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Inverse_kinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Inverse_kinematics)
