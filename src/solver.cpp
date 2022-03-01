#include <iostream>
//#include <Eigen/Eigen>

/* Kinematics & Dynamics Library */
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std;
#define N_JOINT 6

//hi

class RobotSolver
{

public:
  KDL::Tree mytree;
  KDL::Chain mychain;
  KDL::JntArray jointPosition;

  //constructor
  RobotSolver(std::string name)
  {
    initializeVariables();
  }

  ~RobotSolver(void) {}

public:
  void initializeVariables(void)
  {
    jointPosition.resize(N_JOINT);

    //USER INPUT: fill data inside jointPosition
    for (int i = 0; i < N_JOINT; i++)
    {
      jointPosition(i) = 0.0;
    }

    //USER INPUT: parse kdl tree from Urdf
    if (!kdl_parser::treeFromFile("/home/mujib/website/kdl_solver/urdf/model.urdf", mytree))
    {
      cout << "[RobotSolver] Failed to construct kdl tree" << endl;
    }

    if (!mytree.getChain("elfin_base_link", "elfin_end_link", mychain))
    {
      cout << "[RobotSolver] Failed to construct kdl chain" << endl;
    }

    unsigned int nj, ns; // resize variables using # of joints & segments
    nj = mytree.getNrOfJoints();
    ns = mychain.getNrOfSegments();

    if (ns == 0 || nj == 0)
    {
      cout << "[RobotSolver] Number of segments/joints are zero" << endl;
    }
    if (jointPosition.rows() != nj)
    {
      cout << "[RobotSolver] ERROR in size of joint variables" << endl;
    }
  }

  void solveFk()
  {
    KDL::ChainFkSolverPos_recursive fk_solver(mychain);
    double roll, pitch, yaw;
    KDL::Frame _cartpos;
    fk_solver.JntToCart(jointPosition, _cartpos);
    _cartpos.M.GetRPY(roll, pitch, yaw);

    cout << "[RobotSolver] input joint angle: " << jointPosition.data.transpose() << endl;
    cout << "[RobotSolver] ***** output pose *****   " << endl;
    cout << "[RobotSolver] cartesian x-pose: " << _cartpos.p.x() << endl;
    cout << "[RobotSolver] cartesian y-pose: " << _cartpos.p.y() << endl;
    cout << "[RobotSolver] cartesian z-pose: " << _cartpos.p.z() << endl;
    cout << "[RobotSolver] roll: " << roll << endl;
    cout << "[RobotSolver] pitch: " << pitch << endl;
    cout << "[RobotSolver] yaw: " << yaw << endl;
  }
};
int main(int argc, char **argv)
{
  RobotSolver robot("forward kinematics solver");
  robot.solveFk();

  return 0;
}