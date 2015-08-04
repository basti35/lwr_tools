#ifndef OROCOS_INVERSE_KINEMATICS_COMPONENT_HPP
#define OROCOS_INVERSE_KINEMATICS_COMPONENT_HPP

#define USE_PINV     0
#define USE_PINV_NSO 0

#define USE_NR       0
#define USE_NR_JL    0
#define USE_LMA      1


#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/chain.hpp>



#include <kdl/chainfksolverpos_recursive.hpp>


#if USE_LMA
  #include <kdl/chainiksolverpos_lma.hpp>
#endif
#if USE_NR_JL
  #include <kdl/chainiksolverpos_nr_jl.hpp>
#endif
#if USE_NR
  #include <kdl/chainiksolverpos_nr.hpp>
#endif

#if USE_PINV_NSO
  #include <kdl/chainiksolvervel_pinv_nso.hpp>
#endif
#if USE_PINV
  #include <kdl/chainiksolvervel_pinv.hpp>
#endif



#include <kdl/models.hpp>

#include <boost/timer.hpp>

#include <kdl/jntarray.hpp>

#include <rtt/Operation.hpp>

#include <rtt/TaskContext.hpp>

#include <rtt/OperationCaller.hpp>
#include <ocl/OCL.hpp>
#include <ocl/TaskBrowser.hpp>


class Inverse_kinematics : public RTT::TaskContext{
  public:
    Inverse_kinematics(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    KDL::Chain loadKukaLWR();
    KDL::Chain loadPuma260();
	double reshapeAngle(double angle);
    double reshapePosition(double q_old, double q_new);
    KDL::Frame getCartPosition();


    KDL::JntArray q_min_, q_max_, q_init_, q_opt_, weights_;
    KDL::Chain model_;
    KDL::Frame position_;

    Eigen::Matrix<double,6,1> L_;

    std::vector<double> q_min_prop_, q_max_prop_, jnt_pos_double_, weights_prop_;

    RTT::OutputPort <KDL::JntArray> o_jnt_pos_;
    RTT::OutputPort <std::vector<double> > o_jnt_pos_double_;
    RTT::OutputPort <KDL::Frame> o_cart_pos_;
    RTT::OutputPort <std::vector<double> > o_fk_;
    RTT::InputPort <KDL::JntArray> i_rob_pos_;
    RTT::InputPort <KDL::Frame> i_cart_pos_;

    // Solver
    KDL::ChainFkSolverPos_recursive *fk_solver_;

    #if USE_PINV_NSO
        KDL::ChainIkSolverVel_pinv_nso *ik_solver_vel_;
    #endif
    #if USE_PINV
        KDL::ChainIkSolverVel_pinv *ik_solver_vel_;
    #endif

    #if USE_NR_JL
      KDL::ChainIkSolverPos_NR_JL *iksolver_pos_;
    #endif
    #if USE_NR
      KDL::ChainIkSolverPos_NR *iksolver_pos_;
    #endif
    #if USE_LMA
      KDL::ChainIkSolverPos_LMA *iksolver_pos_;
    #endif
};
#endif
