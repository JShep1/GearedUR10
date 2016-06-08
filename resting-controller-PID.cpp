/*****************************************************************************
 * "Controller" for grasping 
 ****************************************************************************/
#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/GravityForce.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/VectorNd.h>
#include <fstream>
#include <stdlib.h>

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

//below are the 24 values for the various perr's, verr's and sum_err of the joints
static double elbow=0;
static double elbowv=0;
static double elbowi=0;
static double shoulderlift = 0;
static double shoulderlifti=0;
static double shoulderliftv = 0;
static double shoulderpan=0;
static double shoulderpanv = 0;
static double shoulderpani = 0;
static double wrist1=0;
static double wrist1i = 0;
static double wrist1v = 0;
static double wrist2=0;
static double wrist2i = 0;
static double wrist2v = 0;
static double wrist3=0;
static double wrist3v = 0;
static double wrist3i = 0;
static double lfinger=0;
static double lfingerv = 0;
static double rfinger=0;
static double rfingeri = 0;
static double lfingeri = 0;
static double rfingerv = 0;
Moby::RCArticulatedBodyPtr robot;
boost::shared_ptr<TimeSteppingSimulator> sim;
boost::shared_ptr<GravityForce> grav;
std::map<std::string, double> q_init;
std::map<std::string, Origin3d> PID_gains;
VectorNd q_des;
bool positionBool = true;

VectorNd& controller(shared_ptr<ControlledBody> body, VectorNd& u, double t, void*)
{
  // setup summed error
  static std::map<std::string, double> summed_err;
  // get the robot body and joints
  boost::shared_ptr<Moby::ArticulatedBody>
  abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(body);
  const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints = abrobot->get_joints();

   // clear the generalized force
  u.set_zero(abrobot->num_generalized_coordinates(DynamicBodyd::eSpatial));
  if (positionBool){
    for (int i = 0; i < joints.size(); i++){
       q_des[i] = joints[i]->q[0];
    }
    positionBool = false;
  } 

  for (unsigned i=0;i< joints.size();i++)
  {
    if (joints[i]->joint_id.find("fixed") == std::string::npos && 
        joints[i]->joint_id != "world_joint")
    {
      // get the desired joint position (always the initial position) 
      double des = q_des[i];

      // get the current joint position and velocity
      double q = joints[i]->q[0];
      double qdot = joints[i]->qd[0];

      //set position error for given joints
      // compute the position error
      if (joints[i]->joint_id == "shoulder_lift_joint"){
	shoulderlift += des-q;
      }else if (joints[i]->joint_id == "elbow_joint"){
	elbow += des-q;
      }else if (joints[i]->joint_id == "shoulder_pan_joint"){
	shoulderpan += des-q;
      }else if (joints[i]->joint_id == "wrist_1_joint"){
	wrist1 += des-q;
      }else if (joints[i]->joint_id == "wrist_2_joint"){
	wrist2 += des-q;
      }else if (joints[i]->joint_id == "wrist_3_joint"){
	wrist3 += des-q;
      }else if (joints[i]->joint_id == "l_finger_actuator"){
	lfinger += des-q;
      }else if (joints[i]->joint_id == "r_finger_actuator"){
	rfinger += des-q;
      }

      double perr = des-q;


      //set respective summed errors for the joints and 
      // get the summed error for the joint
      double& sum_err = summed_err[joints[i]->joint_id];
	if (joints[i]->joint_id == "shoulder_lift_joint"){
	   shoulderliftv += sum_err;
	}else if (joints[i]->joint_id == "elbow_joint"){
	   elbowv += sum_err;
	}else if (joints[i]->joint_id == "shoulder_pan_joint"){
	   shoulderpanv += sum_err;
	}else if (joints[i]->joint_id == "wrist_1_joint"){
	   wrist1v += sum_err;
	}else if (joints[i]->joint_id == "wrist_2_joint"){
	   wrist2v += sum_err;
	}else if (joints[i]->joint_id == "wrist_3_joint"){
	   wrist3v += sum_err;
	}else if (joints[i]->joint_id == "l_finger_actuator"){
	   lfingerv += sum_err;
	}else if (joints[i]->joint_id == "r_finger_actuator"){
	   rfingerv += sum_err;
	}       
      // output the desired and current joint angles (do not modify this)
      std::map<std::string, double>::const_iterator j = q_init.find(joints[i]->joint_id);
      assert(j != q_init.end());
      perr = j->second - joints[i]->q[0];
      std::string fname1 = joints[i]->joint_id + ".desired";
      std::string fname2 = joints[i]->joint_id + ".state";
      std::ofstream out1(fname1.c_str(), std::ostream::app);
      std::ofstream out2(fname2.c_str(), std::ostream::app);
      out1 << j->second << std::endl;
      out2 << joints[i]->q[0] << std::endl;
      out1.close();
      out2.close();

      //set respective joint gains
      // compute the velocity error (note that desired velocity is zero) 
      if (joints[i]->joint_id == "shoulder_lift_joint"){
	   shoulderliftv += -qdot;
	}else if (joints[i]->joint_id == "elbow_joint"){
	   elbowv += -qdot;
	}else if (joints[i]->joint_id == "shoulder_pan_joint"){
	   shoulderpanv += -qdot;
	}else if (joints[i]->joint_id == "wrist_1_joint"){
	   wrist1v += -qdot;
	}else if (joints[i]->joint_id == "wrist_2_joint"){
	   wrist2v += -qdot;
	}else if (joints[i]->joint_id == "wrist_3_joint"){
	   wrist3v += -qdot;
	}else if (joints[i]->joint_id == "l_finger_actuator"){
	   lfingerv += -qdot;
	}else if (joints[i]->joint_id == "r_finger_actuator"){
	   rfingerv += -qdot;
	}       
	double verr = -qdot;
      
      // get the PID gains for the joint
       std::map<std::string, Origin3d>::iterator gainsIt;
      //std::cout <<"id: "<<joints[i]->joint_id<<std::endl;
      gainsIt = PID_gains.find(joints[i]->joint_id);
      double kp = *(gainsIt->second.data(0));
      double ki = *(gainsIt->second.data(1));
      double kv = *(gainsIt->second.data(2)); 
      
      // compute the generalized force contribution
      //set taus for respective joints
      double tau = 0;	
	if (joints[i]->joint_id == "shoulder_lift_joint"){
	tau = (kp*shoulderlift+kv*shoulderliftv+shoulderlifti*ki);
	std::cout<<shoulderlifti<<std::endl;
	}else if(joints[i]->joint_id == "elbow_joint"){
	tau = (kp*elbow+kv*elbowv+ki*elbowi);
	}else if(joints[i]->joint_id == "shoulder_pan_joint"){
	tau = (kp*shoulderpan+kv*shoulderpanv+ki*shoulderpani);
	}else if(joints[i]->joint_id == "wrist_1_joint"){
	tau = (kp*wrist1+kv*wrist1v+ki*wrist1i);
	}else if(joints[i]->joint_id == "wrist_2_joint"){
	tau = (kp*wrist2+kv*wrist2v+ki*wrist2i);
	}else if(joints[i]->joint_id == "wrist_3_joint"){
	tau = (kp*wrist3+kv*wrist3v+ki*wrist3i);
	}else if(joints[i]->joint_id == "l_finger_actuator"){
	tau = (kp*lfinger+kv*lfingerv+ki*lfingeri);
	}else if(joints[i]->joint_id == "r_finger_actuator"){
	tau = (kp*rfinger+kv*rfingerv+ki*rfingeri);
	}else{
	tau = (kp*perr+kv*verr);
	}
      // set the appropriate entry in gf
      u[joints[i]->get_coord_index()] = tau; 

      // update the error sum
      sum_err += perr;
    }
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
    if (i->first == "ur10_schunk_hybrid")
      robot = boost::dynamic_pointer_cast<RCArticulatedBody>(i->second);
  }

  assert(robot);
  robot->controller = &controller; 

  // setup q_init 
  const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints = robot->get_joints();
  robot->get_generalized_coordinates_euler(q_des);
  for (unsigned i=0; i< joints.size(); i++)
    if (joints[i]->joint_id.find("fixed") == std::string::npos && 
        joints[i]->joint_id != "world_joint")
      q_init[joints[i]->joint_id] = q_des[joints[i]->get_coord_index()];

  // overwrite any output files
  for (std::map<std::string, double>::const_iterator i = q_init.begin(); i != q_init.end(); i++)
  {
    const std::string& joint_name = i->first;
    std::string fname1 = joint_name + ".desired";
    std::string fname2 = joint_name + ".state";
    std::ofstream out1(fname1.c_str());
    std::ofstream out2(fname2.c_str());
    out1.close();
    out2.close();
  }

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
  
  std::string lineString;/**Explanation of parsing method is in sinusoidal-controller.cpp**/
  for (int i = 0; i < gainsString.length(); i++){
     char c = gainsString[i];
     if ( (c >64 && c < 91) || (c > 96 && c < 123) || c == 95 || (c>47 && c<58)){
        jointString += c;
     }else if (c == 9){
	while (true){i++; c=gainsString[i]; if (c!=9){break;}}
        while (true){
	   if (c == 9){
	      while (true){i++; c=gainsString[i]; if (c!=9){break;}}
	      if (P){P = false; I = true;}
	      else if (I){I = false; D = true;}
	      else if (D){
		D = false; P = true;
	      }
	   }else if (c == '\n'){
		double pDouble= atof(pNum.c_str());
	   	double iDouble= atof(iNum.c_str());
	   	double dDouble= atof(dNum.c_str());
	   	Ravelin::Origin3d gainValues(pDouble,iDouble,dDouble);
	   	PID_gains.insert(std::pair<std::string,Origin3d>(jointString,gainValues));
		jointString = "";
		pNum = "";               
		iNum = "";		
		dNum = "";
		D = false; P = true;
		break;
	   }
	   if (P){
	        pNum += c;
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

