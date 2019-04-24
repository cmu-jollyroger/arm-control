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
// #include "hebi_cpp_api/util/grav_comp.hpp"
//#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;
using namespace hebi;

class motion
{
public:
	motion();
	KDL::JntArray getIK(geometry_msgs::Pose target_pose);
	void build_traj(geometry_msgs::Pose target_pose);
	//Eigen::MatrixXd interpolateJnts(Eigen::VectorXd cpos, Eigen::VectorXd tpos, int num_wayp);
	bool send_trajectory();
	bool hebi_send_command(Eigen::VectorXd pos);
	Eigen::VectorXd hebi_feedback();

	ros::Publisher joint_pub;

private: 
	KDL::Chain chain;
	std::string chain_start = "base_link";
	std::string chain_end = "end_eff";
	std::string urdf_param = "/robot_description";
	double timeout = 0.06;
	double eps = 0.5e-2;
	KDL::JntArray result;
	KDL::JntArray ll, ul; 

  	
	TRAC_IK::TRAC_IK *tracik_solver;
	TRAC_IK::SolveType type=TRAC_IK::Distance;
	ros::NodeHandle nh;
	KDL::Tree my_tree;
	std::string robot_desc_string;
	std::vector<double> joint_angle_track;
	Eigen::VectorXd homing;

	Lookup lookup;
	std::shared_ptr<Group> group ;
	


};


motion::motion(){



	group = lookup.getGroupFromNames({ "JollyRoger Arm" }, {"Base", "Elbow-1", "Elbow-2", "EndEffector"});
	if (!group)
	{
	std::cout << "Group not found!" << std::endl;
	exit;
	}

	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
	ll.resize(5); ul.resize(5);
	
	ll(0)=-3.14 ; ul(0)=3.14;
	ll(1)=-1.57 ; ul(1)=1.0;
	ll(2)=-3.14 ; ul(2)=3.14;
	ll(3)=-3.14 ; ul(3)=2.75;
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
	joint_angle_track[0] = 0.00909102 ;
	joint_angle_track[1] =-0.411375 ;
	joint_angle_track[2] = 2.74059 ;
	joint_angle_track[3] = 7.90925 ;
	joint_angle_track[4] = 0.0;



	homing.resize(5);
	homing[0] = 0.00909102 ;
	homing[1] =-0.411375 ;
	homing[2] = 2.74059 ;
	homing[3] = 7.90925 ;
	homing[4] = 0.0;
}


KDL::JntArray motion::getIK(geometry_msgs::Pose target_pose){ 
	
  	// valid = tracik_solver->getKDLChain(chain);
	KDL::ChainFkSolverPos_recursive fk_solver(chain);
	KDL::JntArray jnts; 
	jnts.resize(5);
	jnts(0) = 0.00909102 ;
	jnts(1) =-0.411375 ;
	jnts(2) = 2.74059 ;
	jnts(3) = 7.90925 ;
	jnts(4) = 0.0;
	
	KDL::Frame pose;
	fk_solver.JntToCart(jnts,pose);
	double x,y,z,w;

	cout<<pose.p.x() << pose.p.y() <<pose.p.z() <<endl;
	pose.M.GetQuaternion(x,y,z,w); 
	cout<< x << y <<z <<w<<endl;
 
  	KDL::Vector posit(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  	KDL::Rotation orient = KDL::Rotation::Quaternion(target_pose.orientation.x, target_pose.orientation.y,
  													 target_pose.orientation.z, target_pose.orientation.w);

  	KDL::Frame end_effector_pose(orient,posit);
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
	Eigen::VectorXd curr_jnts = hebi_feedback();
	cout<< curr_jnts<<endl;

	return;


}

// Eigen::MatrixXd motion::interpolateJnts(Eigen::VectorXd cpos, Eigen::VectorXd tpos, int num_wayp){ 

// 	for(int i=0; i<cpos.size(); i++ ){

// 		double step_size = (tpos(i) - cpos(i))/2;
// 	}



// }


bool motion::hebi_send_command( Eigen::VectorXd pos){
	
	Eigen::VectorXd efforts(group->size());
	
	efforts.setZero();
	
	hebi::GroupCommand groupCommand(group->size());
	groupCommand.setEffort(efforts);
	groupCommand.setPosition(pos);
	group->sendCommand(groupCommand);
}



Eigen::VectorXd motion::hebi_feedback(){ 

	// This is by default "100"; setting this to 5 here allows the console output
	// to be more reasonable.
	group->setFeedbackFrequencyHz(5);
	GroupFeedback group_fbk(group->size());
	if (group->getNextFeedback(group_fbk))
	{
	  std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
	  return(group_fbk.getPosition());
	  
	}
	group->clearFeedbackHandlers();

}


bool motion::send_trajectory(){

}

int main(int argc, char **argv)
{
	/* code */
	ros::init(argc, argv, "state_publisher");

	ros::start();

	//motion ik; 
	Eigen::VectorXd position; position.resize(6); 
	position<< 0.00909102,
			  -0.411375,
			   2.74059 ,
			   7.90925 ,
			   0.0;
	//ik.hebi_feedback();
	cout<<position<<endl;
	// geometry_msgs::Pose target; 
	// ik.build_traj(target);

	// geometry_msgs::Pose target; 
	// target.position.x =0.03454648;target.position.y =0.35226157;target.position.z = 0.41287656;
	// tf::Quaternion in_q(-0.491329,0.487592,0.513436,0.507182); 





	// //tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI,M_PI/2);


	// target.orientation.x = in_q.x();target.orientation.y = in_q.y();
	// target.orientation.z = in_q.z();target.orientation.w = in_q.w();

	// KDL::JntArray result = ik.getIK(target);
	// printf("Got here_1\n");
	// tf::TransformBroadcaster br;
 //  	tf::Transform transform;
 //  	transform.setOrigin( tf::Vector3(target.position.x, target.position.y, target.position.z) );
 //  	transform.setRotation(in_q);
  	
 //  	printf("Got here_2\n");
  

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
 //    	joint_state.header.stamp = ros::Time::now();
 //    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "pose"));
 //    	ik.joint_pub.publish(joint_state);
 //    	ros::spinOnce();
	//     loop_rate.sleep();
 //    } 


	// ros::spin();
	return 0;
	
}