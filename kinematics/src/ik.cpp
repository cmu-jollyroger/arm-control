#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include <vector>
#include <Eigen/Dense>
//#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;
using namespace hebi;

class motion
{
public:
	motion();
	KDL::JntArray getIK(geometry_msgs::Pose target_pose);
	void build_traj(geometry_msgs::Pose target_pose);
	ros::Publisher joint_pub;
	vector<double> hebi_feedback();
	bool hebi_send_command();
private: 
	KDL::Chain chain;
	std::string chain_start = "base_link";
	std::string chain_end = "end_eff";
	std::string urdf_param = "/robot_description";
	double timeout = 0.05;
	double eps = 1e-3;
	KDL::JntArray result;
	KDL::JntArray ll, ul; 

  	
	TRAC_IK::TRAC_IK *tracik_solver;
	TRAC_IK::SolveType type=TRAC_IK::Distance;
	ros::NodeHandle nh;
	KDL::Tree my_tree;
	std::string robot_desc_string;
	std::vector<double> joint_angle_track;

};


motion::motion(){
	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
	ll.resize(5); ul.resize(5);
	
	ll(0)=-3.14 ; ul(0)=3.14;
	ll(1)=-1.57 ; ul(1)=1.0;
	ll(2)=-3.14 ; ul(2)=3.14;
	ll(3)=-3.14 ; ul(3)=3.14;
	ll(4)=-3.14 ; ul(4)=3.14;

	
	nh.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
	}
	bool got_chain = my_tree.getChain(chain_start,chain_end,chain);
	if(!got_chain){
		ROS_INFO("Unable to load KDL chain");
	}
	tracik_solver = new TRAC_IK::TRAC_IK(chain,ll,ul,timeout, eps,type=type);
	bool valid = tracik_solver->getKDLChain(chain);

  	std::cout << "num of joints in chain: " << chain.getNrOfJoints() << std::endl;


	joint_angle_track.resize(5);
	joint_angle_track[0] = 0.0126214 ;
	joint_angle_track[1] =-0.5649232 ;
	joint_angle_track[2] = 2.78929563 ;
	joint_angle_track[3] = -3.62647655 ;
	joint_angle_track[4] = 0.;
}


KDL::JntArray motion::getIK(geometry_msgs::Pose target_pose){ 
	
  	// valid = tracik_solver->getKDLChain(chain);

  	KDL::Vector posit(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  	KDL::Rotation orient = KDL::Rotation::Quaternion(target_pose.orientation.x, target_pose.orientation.y,
  													 target_pose.orientation.z, target_pose.orientation.w);

  	KDL::Frame end_effector_pose(orient, posit);
	KDL::JntArray nominal(chain.getNrOfJoints());
  
	for (uint j=0; j<nominal.data.size(); j++ ) {
		nominal(j) = joint_angle_track[j];
	}

	int rc=tracik_solver->CartToJnt(nominal,end_effector_pose,result);
	if (rc > 0) {

		std::cout << "ik succeed " << std::endl;
		
		for (uint j=0; j<result.data.size(); j++ ) {
		joint_angle_track[j] = result(j);
		std::cout << result(j) <<std::endl;
		}

		
	}
	else {
		std::cout << "ik failed" << std::endl;
	}
	std::cout << "rc:" << rc << std::endl;


	return(result);

}


void motion::build_traj(geometry_msgs::Pose target_pose){

	//get_current_pose 
	std::vector<double> curr_jnts = hebi_feedback();
	for(int i =0; i<curr_jnts.size(); i++){
		cout << curr_jnts[i] << endl;
	}

	return;


}

bool motion::hebi_send_command(){
	return true;
}

vector<double> motion::hebi_feedback(){ 

	Lookup lookup;
	auto entry_list = lookup.getEntryList();
	auto group = lookup.getGroupFromNames({ "JollyRoger Arm" }, {"Base", "Elbow-1", "Elbow-2", "EndEffector"});
	if (!group)
	{
	std::cout << "Group not found!" << std::endl;
	exit;
	}

	// This is by default "100"; setting this to 5 here allows the console output
	// to be more reasonable.
	group->setFeedbackFrequencyHz(5);
	GroupFeedback group_fbk(group->size());
	vector<double> position; 
	if (group->getNextFeedback(group_fbk))
	{
	  std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
	  Eigen::VectorXd pos = group_fbk.getPosition();
	  position.push_back(pos(0)); 
	  position.push_back(pos(1)); 
	  position.push_back(pos(2));
	  position.push_back(pos(3));
	 
	  return(position);
	  
	}
	

	group->clearFeedbackHandlers();



}

int main(int argc, char **argv)
{
	/* code */
	ros::init(argc, argv, "robot_state_publisher");

	ros::start();
	
	motion ik; 
	geometry_msgs::Pose target; 
	ik.build_traj(target);

	// geometry_msgs::Pose target; 
	// target.position.x = 0.03453803;target.position.y = 0.20218451;target.position.z = 0.16631325;
	// tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI,M_PI/2);
	// target.orientation.x = in_q.x();target.orientation.y = in_q.y();
	// target.orientation.z = in_q.z();target.orientation.w = in_q.w();

	// KDL::JntArray result = ik.getIK(target);
	// printf("Got here_1\n");
	// tf::TransformBroadcaster br;
 //  	tf::Transform transform;
 //  	transform.setOrigin( tf::Vector3(target.position.x, target.position.y, target.position.z) );
 //  	transform.setRotation(in_q);
  	
 //  	printf("Got here_2\n");
  	
 //    // joint_state.position[0] = 0.0;
 //    // joint_state.position[1] = 0.0;
 //    // joint_state.position[2] = 0.0;
 //    // joint_state.position[3] = 0.0;
 //    // joint_state.position[4] = 0.0;

 //  	sensor_msgs::JointState joint_state;
 //  	joint_state.position.resize(5);
 //  	joint_state.name.resize(5);
 //  	joint_state.name[0] = "dummy_to_base";
 //    joint_state.name[1] = "dummy_to_elbow_1";
 //    joint_state.name[2] = "elbow_1_to_elbow_2";
 //    joint_state.name[3] = "elbow_2_to_elbow_3";
 //    joint_state.name[4] = "elbow_3_to_end_eff";

 //    printf("Got here_3\n");
 //    for(int i=0; i<result.data.size(); i++){
 //    	joint_state.position[i] = result(i);
 //    }





 //    ros::Rate loop_rate(10);
 //    printf("Got here_4\n");
 //    //command the robot to move

 //    while (ros::ok()){
 //    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "pose"));
 //    	ik.joint_pub.publish(joint_state);
 //    	ros::spinOnce();
	//     loop_rate.sleep();
 //    } 


	ros::spin();
	return 0;
	
}