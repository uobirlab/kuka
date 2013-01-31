#include <ctime>
#include <string>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "kuka_server.h"
#include "urdf/model.h"
#include <kuka_node/SetTrajectory.h>
#include <kuka_node/ExecuteTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<kuka_node::ExecuteTrajectoryAction> ActionServer;

Kuka::Server server;

// The URDF/Colladda zero position is not aligned with the Birmingham zero.
// Use of an offset to fix
double joint_zero_position[]={0.0, -M_PI_2, M_PI_2, 0.0, 0.0, 0.0};

bool running=true;
void signal_handle(int sig){
//	if (sig==SIGINT) {
		ROS_INFO_STREAM("Ending from signal "<< sig);
		running=false;
		exit(1);
//	}
}

void _setTrajectory(const trajectory_msgs::JointTrajectory_<std::allocator<void> > &new_trajectory){
	//	Initialise a trajectory
	Kuka::TargetJointPos *targets = new Kuka::TargetJointPos[new_trajectory.points.size()];
	for (uint k=0;k<new_trajectory.points.size();k++) {
		for (int i=0; i<6;i++)
			targets[k].joints[i]=(new_trajectory.points[k].positions[i] + joint_zero_position[i])/M_PI * 180.0;
		targets[k].time_to_arrival=new_trajectory.points[k].time_from_start.toSec();
		if (k>0) {
			targets[k].time_to_arrival-=new_trajectory.points[k-1].time_from_start.toSec();
		}
	}
	server.setJointPosTrajectory(targets,new_trajectory.points.size());
	delete targets;
}

bool setTrajectoryService(kuka_node::SetTrajectory::Request  &req,
		kuka_node::SetTrajectory::Response &res ) {

	_setTrajectory(req.new_trajectory);

	res.success.data=true;
	return true;
}

void execute_trajectory_action(const kuka_node::ExecuteTrajectoryGoalConstPtr& goal, ActionServer* as) {
	kuka_node::ExecuteTrajectoryFeedback feedback;
	kuka_node::ExecuteTrajectoryResult result;
	_setTrajectory(goal->trajectory);

	int waypoints=goal->trajectory.points.size();
	float percentage=0;
	while(server.haveTargetPoints()){
		if (as->isPreemptRequested() || !ros::ok())	{
			ROS_INFO("Trajectory execution prempted.");
			// set the action state to preempted
			as->setPreempted();
			server.softStop();
			result.error.data=true;
			as->setSucceeded(result);
			return;
		}
		percentage = 100.0-(server.numberOfWaypoints()/(float)waypoints * 100.0);
		if (feedback.percentage.data!=percentage) {
			feedback.percentage.data=percentage;
			as->publishFeedback(feedback);
		}
	}
	result.error.data=false;
	as->setSucceeded(result);
}

void softStopCallback(const std_msgs::Bool::ConstPtr& msg) {
  ROS_INFO("******************************************************");
  ROS_INFO("Soft-stopping the robot, trajectory will be cancelled.");
  ROS_INFO("******************************************************");
  server.softStop();
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "kuka_node");
	ros::NodeHandle node;

	signal(SIGABRT, &signal_handle);
	signal(SIGTERM, &signal_handle);
	signal(SIGINT, &signal_handle);

	ROS_DEBUG("Starting kuka node...");

	// Initialise the arm model parser and find out what non-fixed joins are present
	urdf::Model armModel;
	std::vector<urdf::Joint*> joints;
	std::map<std::string, unsigned int> nameToNumber;

	armModel.initParam("kuka_description");
	std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator mapElement;
	for (mapElement = armModel.joints_.begin(); mapElement!=armModel.joints_.end(); mapElement++) {
		if ((*mapElement).second.get()->type != urdf::Joint::FIXED)
			joints.push_back((*mapElement).second.get() ); // shared pointer mantaged?
	}
	ROS_INFO("URDF specifies %d non-fixed joints.", joints.size() );

	// Set up the joint state publisher with the joint names
	// This assumes that the joints are ordered on the robot in the same order as
	// the URDF!!!
	//const char* joint_names[]={"A1","A2","A3","A4","A5","A6"};
//	char* joint_names[]={"j0","j1","j2","j3","j4","j5"};
	std::vector<std::string> joint_names;
	if (joints.size() < 6){
		ROS_FATAL("URDF of Kuka doesn't specify enough joints...");
		exit(1);
	}
	// Manages the first 6 joints
	for (unsigned int i=0;i<6;i++) {
		joint_names.push_back(joints[i]->name);
		nameToNumber[joints[i]->name] = i;
		ROS_INFO("Joint %d is mapping to '%s'", i, joints[i]->name.c_str());
	}


	boost::thread server_thread(boost::ref(server));
	ROS_DEBUG("Setting high priority for thread.");
	struct sched_param param;
	pthread_t threadID = (pthread_t) server_thread.native_handle();
	param.sched_priority = 99;
	int err = pthread_setschedparam( threadID,SCHED_RR, &param); // SCHED_RR,
	if (err != 0) {
		ROS_ERROR("Problem setting realtime priority. Perhaps need to be root?");
	}

	ROS_DEBUG("Device server thread active, waiting for kuka to connect.");

	ROS_INFO("Waiting for KUKA connection.");
	while (running && !server.haveClient()) {
		;
	}
//	if (!running) {
//		server.requestShutdown();
//		server_thread.join();
//		ros::Duration w(2.0);
//		w.sleep();
//		if (server_thread.joinable())
//			ROS_DEBUG("Forcing dirty shutdown...");
//		exit(1);
//	}

	ROS_INFO("Communication established.");
	// Loop for publishing joint state messages
	ros::Rate loop_rate(30);
	ros::Publisher state_pub = node.advertise<sensor_msgs::JointState>("kuka_state", 1000);
	ros::ServiceServer service = node.advertiseService("/kuka/set_trajectory", setTrajectoryService);
	sensor_msgs::JointState jointstate;
	jointstate.name.resize(6);
	for (int i=0;i<6;i++)
		jointstate.name[i]=joint_names[i];
	jointstate.effort.resize(6);
	jointstate.position.resize(6);

	// Soft stop message
	ros::Subscriber sub = node.subscribe("/kuka/softstop", 1000, softStopCallback);

	// Action server
	ActionServer action_server(node,"/kuka/execute_trajectory", boost::bind(&execute_trajectory_action, _1, &action_server), false);
	action_server.start();

	// Main loop; publish joint state info.
	while (running) { // && server.haveClient()) {
		Kuka::IncomingDataPacket state = server.getCurrentState();
		for (int i=0; i<6; i++) {
			jointstate.position[i]=state.actualAxisPosition.axis[i]/180.0 * M_PI - joint_zero_position[i];
			jointstate.effort[i]=state.motorCurrents.axis[i];
		}
		jointstate.header.stamp = ros::Time::now(); // server.getRosTimeFromIPOC(state.ipoc);
		state_pub.publish(jointstate);
		ros::spinOnce();
		loop_rate.sleep();
	}
	server.requestShutdown();
	server_thread.join();
	ROS_INFO_STREAM("Attempting to shutdown." << running << ", " << server.haveClient());
	ros::Duration w(3.0);
	w.sleep();
	if (server_thread.joinable())
		ROS_DEBUG("Forcing dirty shutdown...");
	exit(1);


	return 0;
}
