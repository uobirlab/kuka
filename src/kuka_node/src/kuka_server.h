#ifndef KUKASERVER_H_
#define KUKASERVER_H_
#define DEFAULT_PORT 6008
#define BUFFER_SIZE 10240
#define CLOCK_SYNC_CYCLES 200
#define CYCLE_TIME_MS 12

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <queue>
#include <stdlib.h>

namespace Kuka {


struct IncomingDataPacket {
	struct {
		double x,y,z,a,b,c;
	} actualCartPosition, setpointCartPosition ;
	struct {
		double axis[6];
	}  actualAxisPosition, setpointAxisPosition, motorCurrents;
	int delayedPackets; // How many packets have we missed/been late sending - more than 9 is a stop condition....
	// No external axis on ours...
//	struct {
//		float e1,e2,e3,e4,e5,e6;
//	} actualEAxisPosition, setpointEAxisPosition;
	long long int ipoc;

	ros::Time ros_time;
};

enum driveMode_t {CART_VEL,JOINT_VEL,JOINT_POS,STOPPED};

struct OutgoingData {
	char message[300];
	struct {
		double x,y,z,a,b,c;
	} cartCorrection;
	struct {
		double a1,a2,a3,a4,a5,a6;
	} axisCorrection, axisTarget;
	driveMode_t driveMode;

	// The ipoc cylce that we aim for this command to be completed or stopped by
	long long int target_ipoc;
};

struct TargetJointPos{
	float joints[6];
	float time_to_arrival;
};

class Server {
public:
	Server( int portNumber = DEFAULT_PORT);
	virtual ~Server();

	/**
	 *  Entry point for the server thread. Thread started:
	 *    boost::thread(KukaServer())
	 */
	void operator()();


	// Is the Kuka arm connected to the server?
    bool haveClient();

    /**
     * Set the joint position target. Has immediate effect, replaces anything running...
     */
    void setJointPos(TargetJointPos target);

    /**
     * Set a trajectory by filling queue. Replaces anything present.
     */
    void setJointPosTrajectory(TargetJointPos targets[], int targetCount);

    /**
     * Set a message for display in the Kuka KCP.
     */
    void setMessage(const char* msg);

    /**
     * Soft stop the robot: simply cancel the current targets and trajectories.
     */
    void softStop();

    /**
     * Return a copy of the latest state.
     */
    IncomingDataPacket getCurrentState();

    ros::Time getRosTimeFromIPOC(long long int ipoc);
    /**
     * request shutdown
     */
    void requestShutdown();

    /**
     * Is there any target point remaining
     */
    bool haveTargetPoints();

    /**
     * How many waypoints are in the vector of targets.
     */
    int  numberOfWaypoints();

//    void sync

private:
	int portNumber;
	char *incomingDataBuffer; // set to 10K - should be big enough
	char *outgoingDataBuffer;

	double internal_state[6]; // Use this to accumulate sent corrections....
	double correction[6];

	boost::mutex lockIsConnected;
	bool isConnected;

	bool shutdown_req;

	IncomingDataPacket currentState;
	boost::mutex lockCurrentState;
	OutgoingData currentCommand;
	boost::mutex lockCurrentCommand;

	std::queue<TargetJointPos> positionsQueue;
	boost::mutex lockPositionsQueue;

	std::pair<long long int, ros::Time> time_zero;

	bool parseIncomingState( char *xml );
	bool fillOutgoingBuffer();

	static const char* outgoingXMLFormat;

};

template < class RT > RT getTagVal( char *xml, const char *tagname, const char *closetag, RT (*atov)(const char*), char **pos) {
	int tagLength = strlen(tagname);   // could be hard coded...
	char *start = strstr(xml,tagname); // TODO: could start the search nearer to position for more efficiency
	if (start == NULL){
		*pos = NULL;
		return 0;
	}
	char *end = strstr(start+tagLength,closetag);
	if (end == NULL){
		*pos = NULL;
		return 0;
	}
	*end='\0';
	RT ret = atov(start+tagLength);
	*end=closetag[0];
	*pos = end+strlen(closetag);
	return ret;
}



}
#endif /* KUKASERVER_H_ */
