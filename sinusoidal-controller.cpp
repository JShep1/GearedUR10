/*****************************************************************************
 * "Controller" for grasping 
 ****************************************************************************/
#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/RCArticulatedBodyInvDyn.h>
#include <Moby/GravityForce.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/VectorNd.h>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <sstream>
//#define DEBUG

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

using std::vector;
using std::string;
using std::map;
using boost::dynamic_pointer_cast;


static RCArticulatedBodyInvDyn id;
//all static doubles below correspond to the joints pos error and velocity error later used

static double elbow=0;
static double elbowi = 0;
static double shoulderlift = 0;
static double shoulderlifti = 0;
static double elbowv=0;
static double shoulderliftv = 0;
static double shoulderpani = 0;
static double shoulderpan=0;
static double shoulderpanv = 0;
static double wrist1=0;
static double wrist1i = 0;
static double wrist1v = 0;
static double wrist2=0;
static double wrist2i = 0;
static double wrist2v = 0;
static double wrist3=0;
static double wrist3i = 0;
static double wrist3v = 0;
static double lfinger=0;
static double lfingeri = 0;
static double lfingerv = 0;
static double rfinger=0;
static double rfingeri = 0;
static double rfingerv = 0;

static double step_size;
double position_error[8] = {};
double velocity_error[8] = {};
std::vector<double> stator_index;
double dt = -1;
VectorNd tempQ;
std::map<std::string, double> q_init;
std::map<std::string, Origin3d> PID_gains;//first string is the gain label
std::map<std::string, double> q_des, qd_des, qdd_des;//O3D value is the PID gains in that order
Moby::RCArticulatedBodyPtr robot;
boost::shared_ptr<TimeSteppingSimulator> sim;
boost::shared_ptr<GravityForce> grav;

VectorNd& controller(shared_ptr<ControlledBody> body, VectorNd& u, double t, void*)
{
  // get the robot body and joints
  boost::shared_ptr<Moby::ArticulatedBody>
  abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(body);
  const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints = abrobot->get_joints();
  
  std::vector<boost::shared_ptr<Ravelin::Jointd> > stator_joints;
  std::vector<boost::shared_ptr<Ravelin::Jointd> > rotor_joints;
 
  // clear the generalized force
  u.set_zero(abrobot->num_generalized_coordinates(DynamicBodyd::eSpatial));

  // setup the desired positions and velocities
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;

  for (unsigned i=1; i < joints.size(); i++){
      boost::shared_ptr<Ravelin::Jointd> tempJoint = joints[i];
    if ((i-2)%3 == 0 && i<=17 && i > 1){
      stator_joints.push_back(tempJoint);
      stator_index.push_back(i);
    }else if (((i-1)%3 == 0 && i<=16)){
      rotor_joints.push_back(tempJoint);
    }
  }
  for (unsigned i = 0; i < rotor_joints.size(); i++){
     double desP=0;
     double desV=0;
     switch(i){
        case 0:  desP=std::sin(t*PERIOD)*AMP;//base_gear
                 desV=std::cos(t*PERIOD)*AMP*PERIOD;
                 break;
        case 1:  desP=std::sin(t*2.0*PERIOD)*SMALL_AMP;//arm_gear
                 desV=std::cos(t*2.0*PERIOD)*2.0*SMALL_AMP*PERIOD;
                 break;
        case 2:  desP=std::sin(t*(2.0/3.0)*PERIOD)*AMP;//upperarm_gear
                 desV=std::cos(t*(2.0/3.0)*PERIOD)*AMP*PERIOD*(2.0/3.0);
                 break;
        case 3:  desP=std::sin(t*(1.0/7.0)*PERIOD)*AMP;//fore_gear
                 desV=std::cos(t*PERIOD)*AMP*PERIOD*(1.0/7.0);
                 break;
        case 4:  desP=std::sin(t*(2.0/11.0)*PERIOD)*AMP;//wrist1_gear
                 desV=std::cos(t*(2.0/11.0)*PERIOD)*AMP*PERIOD*(2/11.0);
                 break;
        case 5:  desP=std::sin(t*(3.0/13.0)*PERIOD)*AMP;//wrist2_gear
                 desV=std::cos(t*(3.0/13.0)*PERIOD)*AMP*PERIOD*(3.0/13.0);
                 break;
     }
     q_des[rotor_joints[i]->joint_id]=desP;
     qd_des[rotor_joints[i]->joint_id]=desV;
  }
  
  if (rotor_joints.size() == stator_joints.size()){
    for(unsigned i=0; i < rotor_joints.size(); i++){
      double q,qdot,perr,verr,kp,ki,kv,tau;
      q = rotor_joints[i]->q[0];
      qdot = rotor_joints[i]->qd[0];
      std::map<std::string, double>::iterator aq,aqd;
      aq = q_des.find(rotor_joints[i]->joint_id);
      aqd = qd_des.find(rotor_joints[i]->joint_id);
      // compute the position error using desired joint position from 
      // q_des and q
      perr = aq->second - q;
      verr = aqd->second - qdot;
      position_error[i] = perr;
      velocity_error[i] = verr;
      std::map<std::string, Origin3d>::iterator gainsIt;
      std::cout <<"id: "<<rotor_joints[i]->joint_id<<std::endl;
      gainsIt = PID_gains.find(rotor_joints[i]->joint_id);//PID gains holds gains.dat values, parsed in another function
      kp = *(gainsIt->second.data(0));
      ki = *(gainsIt->second.data(1));
      kv = *(gainsIt->second.data(2));
      //std::cout <<"kp: "<<kp<<" ki: "<<ki<<" kd: "<<kv<<std::endl;
      // compute the generalized force contribution
      std::cout <<"perr: "<<position_error[i]<<std::endl;
      std::cout <<"verr: "<<velocity_error[i]<<std::endl;
      tau = 0;
      tau = kp*position_error[i]+kv*velocity_error[i];
      std::cout <<"tau: "<<tau<<std::endl;
      std::map<std::string, double>::const_iterator j = q_init.find(rotor_joints[i]->joint_id);
       //assert(j != q_init.end());
       std::string fname1 = rotor_joints[i]->joint_id + "_desiredPID.txt";
       std::string fname2 = rotor_joints[i]->joint_id + "_statePID.txt";
       std::ofstream out1(fname1.c_str(), std::ostream::app);
       std::ofstream out2(fname2.c_str(), std::ostream::app);
       out1 << 0  << std::endl;
       out2 << q_des[rotor_joints[i]->joint_id] - rotor_joints[i]->q[0] << std::endl;
       out1.close();
       out2.close();
       std::cout<<"length of u"<<u.size()<<"index: "<<stator_joints[i]->get_coord_index()<<std::endl;
         std::cout<<"stored index: "<<stator_index[i]<<std::endl;
       // set the appropriate entry in gf
       for (unsigned j = 0; j < stator_joints.size(); j++){

         std::cout<<stator_joints[i]->joint_id<<std::endl;
       }
       u[stator_index[i]-2] = tau; 
    }
  }else{
    std::cout<<"Unequal amount of rotor and stator joints, PD control cannot be performed"<<std::endl;
  }
  return u; 
}

/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  const unsigned Z = 2;

  // get a reference to the TimeSteppingSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<TimeSteppingSimulator>(i->second);
    if (i->first == "ur10")
      robot = boost::dynamic_pointer_cast<RCArticulatedBody>(i->second);
  }

  assert(robot);
  robot->controller = &controller; 
  
  // sets the starting velocity for the robot joints (do not modify this)
  const std::vector<shared_ptr<Jointd> >& joints = robot->get_joints();
    robot->get_generalized_coordinates_euler(tempQ);
  for (unsigned i=0; i< joints.size(); i++)
    if (joints[i]->joint_id.find("fixed") == std::string::npos && 
        joints[i]->joint_id != "world_joint")
      q_init[joints[i]->joint_id] = tempQ[joints[i]->get_coord_index()];


  
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP * 0.1;
  std::map<std::string, double> qd_init;
  qd_init["base_gear_joint"] = AMP*PERIOD;
  qd_init["arm_gear_joint"] = SMALL_AMP*PERIOD*2.0;
  qd_init["elbow_joint"] = AMP*PERIOD*2.0/3.0;
  qd_init["upperarm_gear_joint"] = AMP*PERIOD*1.0/7.0;
  qd_init["wrist1_gear_joint"] = AMP*PERIOD*2.0/11.0;
  qd_init["wrist2_gear_joint"] = AMP*PERIOD*3.0/13.0;
  qd_init["l_finger_actuator"] = 0.0;
  qd_init["r_finger_actuator"] = 0.0;
  VectorNd qd;
  robot->get_generalized_velocity(DynamicBodyd::eEuler, qd);
  for (unsigned i=0; i< joints.size(); i++)
    qd[joints[i]->get_coord_index()] = qd_init[joints[i]->joint_id];
  robot->set_generalized_velocity(DynamicBodyd::eEuler, qd);
  
  // read gains from the gains file
  std::ifstream in("gains.dat");
  if (in.fail())
  {
    std::cerr << "Failed to open 'gains.dat' for reading!" << std::endl;
    exit(-1);
  }
  char c;bool stringbool=true; bool P=true; bool I=false; bool D = false;
  bool done = false;
  std::string gainsString;
  std::string jointString; 
  std::string Num;
  std::string pNum; 
  std::string iNum; 
  std::string dNum;
  int i = 0;
  std::stringstream ss;
  while(in.get(c)){
   ss << c;
  }
  gainsString = ss.str();
  
  
  // overwrite any output files
  for (std::map<std::string, double>::const_iterator i = q_init.begin(); i != q_init.end(); i++)
  {
    const std::string& joint_name = i->first;
    std::string fname1 = joint_name + "_desiredPID.txt";
    std::string fname2 = joint_name + "_statePID.txt";
    std::ofstream out1(fname1.c_str());
    std::ofstream out2(fname2.c_str());
    out1.close();
    out2.close();
  }



  std::string lineString;
  // read the step size
  char* stepsize_str = getenv("STEP_SIZE");
  //std::cout << stepsize_str<<std::endl;
  //float stepsize_str = getenv("STEP_SIZE");
  
  if (!stepsize_str)
    throw std::runtime_error("No STEP_SIZE variable specified!");
  step_size = std::atof(stepsize_str);
  //std::cout << step_size << std::endl;

  for (int i = 0; i < gainsString.length(); i++){
     char c = gainsString[i];//this char is the one that is iterated through in the text file
     if ( (c >64 && c < 91) || (c > 96 && c < 123) || c == 95 || (c>47 && c<58)){//corresponds to the available
        jointString += c;  //ASCII values for letters and numbers, and hyphens
     }else if (c == 9){//if a tab is found, means that a number is coming
	while (true){i++; c=gainsString[i]; if (c!=9){break;}}//skip all tabs until a number is found
        while (true){
	   if (c == 9){//if a another tab is found that means its the next number
	      while (true){i++; c=gainsString[i]; if (c!=9){break;}}//skip thru tabs
	      if (P){P = false; I = true;}//if its time for p gain, we know i gain is next so set bool
	      else if (I){I = false; D = true;}
	      else if (D){
		D = false; P = true;
	      }
	   }else if (c == '\n'){
		double pDouble= atof(pNum.c_str());//if a new line is found we know we have obtained all values
	   	double iDouble= atof(iNum.c_str());//so turn the strings into doubles
	   	double dDouble= atof(dNum.c_str());
	   	Ravelin::Origin3d gainValues(pDouble,iDouble,dDouble);//created an O3D of gains
	   	PID_gains.insert(std::pair<std::string,Origin3d>(jointString,gainValues));//insert into map
		jointString = "";//reset all found values to begin finding the next row
		pNum = "";               
		iNum = "";		
		dNum = "";
		D = false; P = true;
		break;
	   }
	   if (P){
	        pNum += c;//if its time for a p gain to be entered, add the sucker in
	   }else if (D){
		dNum += c;
	   }else if (I){
		iNum += c;
	   }

	   i++;
	   c = gainsString[i];
        }
     }
  }
}
} // end extern C

