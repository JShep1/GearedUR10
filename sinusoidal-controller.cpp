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

  // clear the generalized force
  u.set_zero(abrobot->num_generalized_coordinates(DynamicBodyd::eSpatial));

  // setup the desired positions and velocities
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  q_des["shoulder_pan_joint"] = std::sin(t*PERIOD)*AMP;
  q_des["shoulder_lift_joint"] = std::sin(t*2.0*PERIOD)*SMALL_AMP;
  q_des["elbow_joint"] = std::sin(t*2.0/3.0*PERIOD)*AMP;
  q_des["wrist_1_joint"] = std::sin(t*1.0/7.0*PERIOD)*AMP;
  q_des["wrist_2_joint"] = std::sin(t*2.0/11.0*PERIOD)*AMP;
  q_des["wrist_3_joint"] = std::sin(t*3.0/13.0*PERIOD)*AMP;
  q_des["l_finger_actuator"] = 0.0;
  q_des["r_finger_actuator"] = 0.0;

  qd_des["shoulder_pan_joint"] = std::cos(t*PERIOD)*AMP*PERIOD;
  qd_des["shoulder_lift_joint"] = std::cos(t*2.0*PERIOD)*SMALL_AMP*PERIOD*2.0;
  qd_des["elbow_joint"] = std::cos(t*2.0/3.0*PERIOD)*AMP*PERIOD*2.0/3.0;
  qd_des["wrist_1_joint"] = std::cos(t*1.0/7.0*PERIOD)*AMP*PERIOD*1.0/7.0;
  qd_des["wrist_2_joint"] = std::cos(t*2.0/11.0*PERIOD)*AMP*PERIOD*2.0/11.0;
  qd_des["wrist_3_joint"] = std::cos(t*3.0/13.0*PERIOD)*AMP*PERIOD*3.0/13.0;
  qd_des["l_finger_actuator"] = 0.0;
  qd_des["r_finger_actuator"] = 0.0;


  for (unsigned i=0;i< joints.size();i++)
  {
    if (joints[i]->joint_id.find("fixed") == std::string::npos && 
        joints[i]->joint_id != "world_joint")
    { double q = joints[i]->q[0];
      double qdot = joints[i]->qd[0];
	std::map<std::string, double>::iterator aq,aqd;
	aq = q_des.find(joints[i]->joint_id);
	aqd = qd_des.find(joints[i]->joint_id);
      // compute the position error using desired joint position from 
      // q_des and q
      double perr = aq->second - q;

	if (joints[i]->joint_id == "shoulder_lift_joint"){//the following set the respective joints'
	shoulderlift = perr;
	shoulderlifti +=perr;
      }else if (joints[i]->joint_id == "elbow_joint"){
	elbow = perr;
	elbowi+= perr;
      }else if (joints[i]->joint_id == "shoulder_pan_joint"){
	shoulderpan = perr;
	shoulderpani +=perr;
      }else if (joints[i]->joint_id == "wrist_1_joint"){
	wrist1 = perr;
	wrist1i +=perr;
      }else if (joints[i]->joint_id == "wrist_2_joint"){
	wrist2 = perr;
	wrist2i += perr;
      }else if (joints[i]->joint_id == "wrist_3_joint"){
	wrist3 = perr;
	wrist3i += perr;
      }else if (joints[i]->joint_id == "l_finger_actuator"){
	lfinger = perr;
	lfingeri += perr;
      }else if (joints[i]->joint_id == "r_finger_actuator"){
	rfinger = perr;
	rfingeri += perr;
      }
//compute the velocity error using desired joint velocity from 
      // qd_des and q
      double verr = aqd->second - qdot;
       if (joints[i]->joint_id == "shoulder_lift_joint"){//set the joints' respective velocity error
	shoulderliftv = verr;
      }else if (joints[i]->joint_id == "elbow_joint"){
	elbowv = verr;
      }else if (joints[i]->joint_id == "shoulder_pan_joint"){
	shoulderpanv = verr;
      }else if (joints[i]->joint_id == "wrist_1_joint"){
	wrist1v = verr;
      }else if (joints[i]->joint_id == "wrist_2_joint"){
	wrist2v = verr;
      }else if (joints[i]->joint_id == "wrist_3_joint"){
	wrist3v = verr;
      }else if (joints[i]->joint_id == "l_finger_actuator"){
	lfingerv = verr;
      }else if (joints[i]->joint_id == "r_finger_actuator"){
	rfingerv = verr;
      }
      // get the  gains for the joint
      std::map<std::string, Origin3d>::iterator gainsIt;
      //std::cout <<"id: "<<joints[i]->joint_id<<std::endl;
      gainsIt = PID_gains.find(joints[i]->joint_id);//PID gains holds gains.dat values, parsed in another function
      double kp = *(gainsIt->second.data(0));
      double ki = *(gainsIt->second.data(1));
      double kv = *(gainsIt->second.data(2));
	//std::cout <<"kp: "<<kp<<" ki: "<<ki<<" kd: "<<kv<<std::endl;
      // compute the generalized force contribution
      double tau = 0;	
	if (joints[i]->joint_id == "shoulder_lift_joint"){//set the tau value for the given joint
	  tau = (kp*shoulderlift+kv*shoulderliftv);
	}else if(joints[i]->joint_id == "elbow_joint"){
	  tau = (kp*elbow+kv*elbowv);
	}else if(joints[i]->joint_id == "shoulder_pan_joint"){
	  tau = (kp*shoulderpan+kv*shoulderpanv);
	}else if(joints[i]->joint_id == "wrist_1_joint"){
	  tau = (kp*wrist1+kv*wrist1v);
	}else if(joints[i]->joint_id == "wrist_2_joint"){
	  tau = (kp*wrist2+kv*wrist2v);
	}else if(joints[i]->joint_id == "wrist_3_joint"){
	  tau = (kp*wrist3+kv*wrist3v);
	}else if(joints[i]->joint_id == "l_finger_actuator"){
	  tau = (kp*lfinger+kv*lfingerv);
	}else if(joints[i]->joint_id == "r_finger_actuator"){
	  tau = (kp*rfinger+kv*rfingerv);
	}else{
  	  tau = (kp*perr+kv*verr);
	}
	
	std::map<std::string, double>::const_iterator j = q_init.find(joints[i]->joint_id);
        //assert(j != q_init.end());
       
        std::string fname1 = joints[i]->joint_id + "_desiredPID.txt";
        std::string fname2 = joints[i]->joint_id + "_statePID.txt";
        std::ofstream out1(fname1.c_str(), std::ostream::app);
        std::ofstream out2(fname2.c_str(), std::ostream::app);

        out1 << 0  << std::endl;
        out2 << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        out1.close();
        out2.close();
        // set the appropriate entry in gf
        u[joints[i]->get_coord_index()] = tau; 
    }
  }


  return u; 
}
/*
VectorNd& controller(shared_ptr<ControlledBody> body, VectorNd& u, double t, void*)
{
  if (dt != -1 && t != 0){
    dt = t;//set up dt for later use in error control
  }
  // get the robot body and joints
  boost::shared_ptr<Moby::RCArticulatedBody>
  abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(body);
  const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints = abrobot->get_joints();
  RCArticulatedBodyInvDynData idyn_data;

  // clear the generalized force
  u.set_zero(abrobot->num_generalized_coordinates(DynamicBodyd::eSpatial));
  idyn_data.qdd_des.set_zero(abrobot->num_joint_dof_explicit());
  

  // setup the desired positions and velocities
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  const double SCALE_ERROR = 10;
  const double SCALE_V_ERROR = 1;
  q_des["shoulder_pan_joint"] = std::sin(t*PERIOD)*AMP;
  q_des["shoulder_lift_joint"] = std::sin(t*2.0*PERIOD)*SMALL_AMP;
  q_des["elbow_joint"] = std::sin(t*2.0/3.0*PERIOD)*AMP;
  q_des["wrist_1_joint"] = std::sin(t*1.0/7.0*PERIOD)*AMP;
  q_des["wrist_2_joint"] = std::sin(t*2.0/11.0*PERIOD)*AMP;
  q_des["wrist_3_joint"] = std::sin(t*3.0/13.0*PERIOD)*AMP;
  q_des["r_finger_actuator"] = 0.0;
  q_des["l_finger_actuator"] = 0.0;

  qd_des["shoulder_pan_joint"] = std::cos(t*PERIOD)*AMP*PERIOD;
  qd_des["shoulder_lift_joint"] = std::cos(t*2.0*PERIOD)*SMALL_AMP*PERIOD*2.0;
  qd_des["elbow_joint"] = std::cos(t*2.0/3.0*PERIOD)*AMP*PERIOD*2.0/3.0;
  qd_des["wrist_1_joint"] = std::cos(t*1.0/7.0*PERIOD)*AMP*PERIOD*1.0/7.0;
  qd_des["wrist_2_joint"] = std::cos(t*2.0/11.0*PERIOD)*AMP*PERIOD*2.0/11.0;
  qd_des["wrist_3_joint"] = std::cos(t*3.0/13.0*PERIOD)*AMP*PERIOD*3.0/13.0;
  qd_des["l_finger_actuator"] = 0.0;
  qd_des["r_finger_actuator"] = 0.0;

  //set error feedback terms below
  for(unsigned i=0;i<joints.size();i++){
  if (i == 1){//shoulder pan
        shoulderpani = q_des[joints[i]->joint_id] - joints[i]->q[0];
	shoulderpanv = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "shoulder pan error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 2){//shoulder lift
         shoulderlifti = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 shoulderliftv = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "shoulder lift error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 3){//elbow
         elbowi = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 elbowv = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "elbow error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 4){//wrist 1
         wrist1i = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 wrist1v = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "wrist 1 error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 5){//wrist 2
         wrist2i = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 wrist2v = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "wrist 2 error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 6){//wrist 3
         wrist3i = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 wrist3v = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "wrist 2 error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 8){//left finger
         lfingeri = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 lfingerv = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "left finger error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }else if (i == 9){//right finger
         rfingeri = q_des[joints[i]->joint_id] - joints[i]->q[0];
	 rfingerv = qd_des[joints[i]->joint_id] - joints[i]->qd[0];
        #ifdef DEBUG
          std::cout << "right finger error: " << q_des[joints[i]->joint_id] - joints[i]->q[0] << std::endl;
        #endif
      }

    }

  //the joint variables ending in "i" are the error feedback terms
  qdd_des["shoulder_pan_joint"] = std::sin(t*PERIOD)*AMP*PERIOD*PERIOD*(-1)+SCALE_ERROR*40*shoulderpani+4*SCALE_V_ERROR*shoulderpanv;

  qdd_des["shoulder_lift_joint"] = std::sin(t*2.0*PERIOD)*SMALL_AMP*PERIOD*PERIOD*(-4.0)+SCALE_ERROR*115*shoulderlifti+SCALE_V_ERROR*10*shoulderliftv;

  qdd_des["elbow_joint"] = std::sin(t*2.0/3.0*PERIOD)*AMP*PERIOD*PERIOD*(-4.0/9.0)+SCALE_ERROR*50*elbowi+SCALE_V_ERROR*20*elbowv;

  qdd_des["wrist_1_joint"] = std::sin(t*1.0/7.0*PERIOD)*AMP*PERIOD*PERIOD*(-1.0/49.0)+SCALE_ERROR*30*wrist1i+SCALE_V_ERROR*10*wrist1v;

  qdd_des["wrist_2_joint"] = std::sin(t*2.0/11.0*PERIOD)*AMP*PERIOD*PERIOD*(-4.0/121.0)+SCALE_ERROR*30*wrist2i+SCALE_V_ERROR*wrist2v*10;

  qdd_des["wrist_3_joint"] = std::sin(t*3.0/13.0*PERIOD)*AMP*PERIOD*PERIOD*(-9.0/169.0)+SCALE_ERROR*40*wrist3i+SCALE_V_ERROR*wrist3v*10;

  qdd_des["l_finger_actuator"] = lfingeri*20*SCALE_ERROR+SCALE_V_ERROR*30*lfingerv;
  qdd_des["r_finger_actuator"] = rfingeri*SCALE_ERROR+SCALE_V_ERROR*rfingerv;

  #ifdef DEBUG
  std::cout << "shoulder pan: " << shoulderpani << std::endl;
  std::cout << "shoulder lift: " << shoulderlifti << std::endl;
  std::cout << "elbow: " << elbowi << std::endl;
  std::cout << "wrist 1: " << wrist1i << std::endl;
  std::cout << "wrist 2: " << wrist2i << std::endl;
  std::cout << "wrist 3: " << wrist3i << std::endl;
  std::cout << "left finger: " << lfingeri << std::endl;
  std::cout << "right finger: " << rfingeri << std::endl;

  #endif
//take error that im accumulating and feed it back into the acceleration command


  //abrobot->set_generalized_coordinates_euler(q_des);
  //abrobot->set_generalized_velocity(DynamicBodyd::eSpatial, qd_des);
  const vector<shared_ptr<RigidBodyd> >& links = abrobot->get_links();  
  for(unsigned i=0;i<joints.size();i++){
    if(joints[i]->num_dof() != 0){
      
      
      idyn_data.qdd_des[joints[i]->get_coord_index()] = qdd_des[joints[i]->joint_id];
      
      std::map<std::string, double>::const_iterator j = q_init.find(joints[i]->joint_id);
      //assert(j != q_init.end());
      
      std::string fname1 = joints[i]->joint_id + "_desired05.txt";
      std::string fname2 = joints[i]->joint_id + "_state05.txt";
      std::ofstream out1(fname1.c_str(), std::ostream::app);
      std::ofstream out2(fname2.c_str(), std::ostream::app);

      //out1 << q_des[joints[i]->joint_id] << " " << std::endl;
      //out2 << joints[i]->q[0] << " " << std::endl;
      out1 << q_des[joints[i]->joint_id] << " " << std::endl;
      out2 << joints[i]->q[0]<< " " << std::endl;

      out1.close();

      out2.close();
      
    }
  }
  id.calc_inv_dyn(abrobot, idyn_data, step_size, u);


  
  


  return u; 
}*/





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
    if (i->first == "ur10_schunk_hybrid")
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
  qd_init["shoulder_pan_joint"] = AMP*PERIOD;
  qd_init["shoulder_lift_joint"] = SMALL_AMP*PERIOD*2.0;
  qd_init["elbow_joint"] = AMP*PERIOD*2.0/3.0;
  qd_init["wrist_1_joint"] = AMP*PERIOD*1.0/7.0;
  qd_init["wrist_2_joint"] = AMP*PERIOD*2.0/11.0;
  qd_init["wrist_3_joint"] = AMP*PERIOD*3.0/13.0;
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

