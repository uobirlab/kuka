/*
 * KukaServer.cpp
 *
 *  Created on: 10 Nov 2011
 *      Author: burbrcjc
 */

#include "kuka_server.h"
#include <time.h>
#include <string.h>
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>

using boost::asio::ip::tcp;
using namespace Kuka;

const char* Server::outgoingXMLFormat =
"<Sen Type=\"KukaAlive\"> \n\
<EStr>%s</EStr> \n\
<RKorr X=\"%g\" Y=\"%g\" Z=\"%g\" A=\"%g\" B=\"%g\" C=\"%g\" /> \n\
<AKorr A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
<IPOC>%d</IPOC> \n\
</Sen> \n\
";
const double SEC_PER_IPO = 12.0 / 1000.0;
const double IPO_PER_SEC = 1000.0 / 12.0;

Server::Server(int portNumber)  {
	this->isConnected = false; // not connected to begin with
	this->portNumber = portNumber;

	shutdown_req=false;

	// Deafult size 10K
	incomingDataBuffer = new char[BUFFER_SIZE];

	// Formatted XML output buffer
	outgoingDataBuffer = new char[BUFFER_SIZE];

	currentCommand.axisCorrection.a1 = currentCommand.axisCorrection.a2 = currentCommand.axisCorrection.a3 =
			currentCommand.axisCorrection.a4 = currentCommand.axisCorrection.a5 = currentCommand.axisCorrection.a6 = 0;

	currentCommand.cartCorrection.x = currentCommand.cartCorrection.y = currentCommand.cartCorrection.z =
			currentCommand.cartCorrection.a = currentCommand.cartCorrection.b = currentCommand.cartCorrection.c = 0;

	setMessage("Initiating comms.");
	currentCommand.driveMode = STOPPED;

	for (int i=0;i<6;i++){
		correction[i]=0.0;
		internal_state[i]=0.0;
	}
}

Server::~Server() {
	delete incomingDataBuffer;
	delete outgoingDataBuffer;
}

void Server::operator ()() {
	// Start listening on the TCP port, receive positions and respond
	// with corrections.

	try {
		boost::asio::io_service io_service;

		tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), this->portNumber));

		while (true && !shutdown_req) {
			tcp::socket socket(io_service);
			{
				boost::unique_lock<boost::mutex> lock(lockIsConnected);
				this->isConnected = false;
			}

			boost::thread::yield();

			ROS_INFO("Acceptor wating for connection...");
			acceptor.accept(socket);


			// A connection is established so do the stuff
			boost::system::error_code error;
			size_t pos=0;

			// For first 100 packets measure the sync between ros::Time and ipoc...
			ROS_INFO("Syncing clocks.");
			std::vector<std::pair<long long int, ros::Time> > clock_sync;
			clock_sync.resize(CLOCK_SYNC_CYCLES);
			for (int i=0; i<CLOCK_SYNC_CYCLES; i++) {
				// Read an XML package...
				size_t len = socket.read_some( boost::asio::buffer(incomingDataBuffer + pos, BUFFER_SIZE - pos), error);
				pos += len;

				incomingDataBuffer[pos+1] = '\0'; // stop the string search at the end
				char *location = strstr((char*)&(incomingDataBuffer[0]), "</Rob>");

				if (location!=NULL) {
					pos = 0;
					// Extract the useful informations...
					this->parseIncomingState(incomingDataBuffer);
					clock_sync[i]=std::pair<long long int, ros::Time>(currentState.ipoc,ros::Time::now());
//					ROS_INFO_STREAM("Syn Time: " << clock_sync[i].second << "s");

					// Send blank data to the robot
					setMessage("Syncing clocks....");
					sprintf(outgoingDataBuffer, outgoingXMLFormat,
									currentCommand.message,
									0.0, 0.0, 0.0,  0.0, 0.0, 0.0, // zero cartesian corrections...
									0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
									currentState.ipoc
									);

					std::string outgoingDatamessage = outgoingDataBuffer;/// YuehChuan
					boost::system::error_code write_error;
////					boost::asio::write(socket,boost::asio::buffer(outgoingDataBuffer),boost::asio::transfer_all(), write_error);   /origin
					boost::asio::write(socket, boost::asio::buffer(outgoingDatamessage), boost::asio::transfer_all(), write_error); /// YuehChuan
				}
			}
			// The diff between each should be 12ms, anything more than that is the jitter. Calculate
			// average jitter
			ros::Duration jitter(0);
			for (int i=0;i<CLOCK_SYNC_CYCLES-1;i++){
				jitter += (clock_sync[i+1].second - clock_sync[i].second);
			}
			double jitter_seconds = jitter.toSec() / (CLOCK_SYNC_CYCLES-1);
			jitter_seconds -= CYCLE_TIME_MS/1000.0;

			jitter = jitter.fromSec((jitter.toSec() / (CLOCK_SYNC_CYCLES - 1))-(CYCLE_TIME_MS/1000.0) );
//			jitter -= ros::Duration(0,CYCLE_TIME_MS*1000);
//			ros::Duration d(jitter_seconds);
			ROS_INFO_STREAM("Average jitter: " << jitter << "s");
			ROS_INFO_STREAM("Last time: " << clock_sync[CLOCK_SYNC_CYCLES-1].second);

			time_zero = std::pair<long long int, ros::Time>(clock_sync.end()->first,
					clock_sync[CLOCK_SYNC_CYCLES-1].second -jitter );
			for (int i=0;i<6;i++)
				internal_state[i]=currentState.actualAxisPosition.axis[i];
			//std::pair<long long int, ros::Time>(clock_sync.end()->first,
				//	clock_sync.end()->second - jitter );

			// Clear any trajectory...
			while (!positionsQueue.empty()) {
				positionsQueue.pop();
			}
			currentCommand.driveMode=STOPPED;


			{
				boost::unique_lock<boost::mutex> lock(lockIsConnected);
				this->isConnected = true;
			}
			setMessage("Ready.");
//			while ( && !shutdown_req) { //while connected.

			while (socket.is_open() && !shutdown_req) { //while connected.
				// Read an XML package...
				size_t len = socket.read_some( boost::asio::buffer(incomingDataBuffer + pos, BUFFER_SIZE - pos), error);
				if (error.value()!=boost::system::errc::success) {
					ROS_INFO("Conection to Kuka died. ?");
					socket.close();
					break;
				}
				pos += len;

				incomingDataBuffer[pos+1] = '\0'; // stop the string search at the end
				char *location = strstr((char*)&(incomingDataBuffer[0]), "</Rob>");

				if (location!=NULL) {
//					 reached end of robot xml blip
//					if (location + 8 - incomingData == pos) {
						// safe to return to buffer beginning....
						pos = 0;
//					} // TODO: This should be fine, but just in case need to check for stray

					// Extract the useful informations...
					this->parseIncomingState(incomingDataBuffer);


					// Send data to the robot
					this->fillOutgoingBuffer();
					boost::system::error_code write_error;
					std::string outgoingDatamessage = outgoingDataBuffer;/// YuehChuan
////					boost::asio::write(socket,boost::asio::buffer(outgoingDataBuffer),boost::asio::transfer_all(), write_error);

							boost::asio::write(socket, boost::asio::buffer(outgoingDatamessage),boost::asio::transfer_all(), write_error); /// YuehChuan
					if (write_error.value()!=boost::system::errc::success) {
						ROS_INFO("Conection to Kuka died. ?");
						socket.close();
						break;
					}
				}
			}
		}
	} catch (std::exception& e) {
		// What?
		std::cerr << e.what() << std::endl;
	}

}

bool Server::haveClient() {
	boost::unique_lock<boost::mutex> lock(lockIsConnected);
    return this->isConnected;
}


bool Server::parseIncomingState( char *xml ) {
	// Go over the string to fill in the currentState
	boost::unique_lock<boost::mutex> lock(lockCurrentState);
	static 	int last_delayed_packets=0;


	char *position;

	currentState.actualCartPosition.x = Kuka::getTagVal<double>(xml, "<RIst X=\"",  "\"",  &atof, &position);
	currentState.actualCartPosition.y = Kuka::getTagVal<double>(position, "Y=\"",  "\"",  &atof, &position);
	currentState.actualCartPosition.z = Kuka::getTagVal<double>(position, "Z=\"",  "\"",  &atof, &position);
	currentState.actualCartPosition.a = Kuka::getTagVal<double>(position, "A=\"",  "\"",  &atof, &position);
	currentState.actualCartPosition.b = Kuka::getTagVal<double>(position, "B=\"",  "\"",  &atof, &position);
	currentState.actualCartPosition.c = Kuka::getTagVal<double>(position, "C=\"",  "\"",  &atof, &position);

	currentState.setpointCartPosition.x = Kuka::getTagVal<double>(position, "<RSol X=\"",  "\"",  &atof, &position);
	currentState.setpointCartPosition.y = Kuka::getTagVal<double>(position, "Y=\"",  "\"",  &atof, &position);
	currentState.setpointCartPosition.z = Kuka::getTagVal<double>(position, "Z=\"",  "\"",  &atof, &position);
	currentState.setpointCartPosition.a = Kuka::getTagVal<double>(position, "A=\"",  "\"",  &atof, &position);
	currentState.setpointCartPosition.b = Kuka::getTagVal<double>(position, "B=\"",  "\"",  &atof, &position);
	currentState.setpointCartPosition.c = Kuka::getTagVal<double>(position, "C=\"",  "\"",  &atof, &position);

	currentState.actualAxisPosition.axis[0] = Kuka::getTagVal<double>(position, "<AIPos A1=\"",  "\"",  &atof, &position);
	currentState.actualAxisPosition.axis[1] = Kuka::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState.actualAxisPosition.axis[2] = Kuka::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState.actualAxisPosition.axis[3] = Kuka::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState.actualAxisPosition.axis[4] = Kuka::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState.actualAxisPosition.axis[5] = Kuka::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

	currentState.setpointAxisPosition.axis[0] = Kuka::getTagVal<double>(position, "<ASPos A1=\"",  "\"",  &atof, &position);
	currentState.setpointAxisPosition.axis[1] = Kuka::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState.setpointAxisPosition.axis[2] = Kuka::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState.setpointAxisPosition.axis[3] = Kuka::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState.setpointAxisPosition.axis[4] = Kuka::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState.setpointAxisPosition.axis[5] = Kuka::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

	currentState.motorCurrents.axis[0] = Kuka::getTagVal<double>(position, "<MACur A1=\"",  "\"",  &atof, &position);
	currentState.motorCurrents.axis[1] = Kuka::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState.motorCurrents.axis[2] = Kuka::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState.motorCurrents.axis[3] = Kuka::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState.motorCurrents.axis[4] = Kuka::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState.motorCurrents.axis[5] = Kuka::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

	currentState.delayedPackets = Kuka::getTagVal<long long int>(xml, "<Delay D=\"",  "\" />",  &atoll, &position);

	if (currentState.delayedPackets != last_delayed_packets){
		ROS_WARN_STREAM("Dropped " << currentState.delayedPackets << "packets!");
		// last correction was not applied then?
		for (int i=0;i<6;i++)
			internal_state[i]-=correction[i];
	}

	currentState.ipoc = Kuka::getTagVal<long long int>(xml, "<IPOC>",  "</IPOC>",  &atoll, &position);


	return true;
}


void Kuka::Server::setJointPos(TargetJointPos target) { //float a1, float a2, float a3, float a4, float a5, float a6, float duration) {
  //	ROS_INFO("Setting target point.");
	boost::unique_lock<boost::mutex> lock1(lockCurrentState);
	boost::unique_lock<boost::mutex> lock2(lockCurrentCommand);
	currentCommand.driveMode = JOINT_POS;
	currentCommand.axisTarget.a1 = target.joints[0];
	currentCommand.axisTarget.a2 = target.joints[1];
	currentCommand.axisTarget.a3 = target.joints[2];
	currentCommand.axisTarget.a4 = target.joints[3];
	currentCommand.axisTarget.a5 = target.joints[4];
	currentCommand.axisTarget.a6 = target.joints[5];
	currentCommand.target_ipoc = currentState.ipoc + target.time_to_arrival*(IPO_PER_SEC);
	//	ROS_INFO("Target point set");
}

void Kuka::Server::setJointPosTrajectory(TargetJointPos targets[], int targetCount) {
	if (targetCount<1){
		ROS_WARN("Setting joint trajectory with no targets!");
		softStop();
		return;
	}
	{
		boost::unique_lock<boost::mutex> lock1(lockCurrentState);
		boost::unique_lock<boost::mutex> lock2(lockCurrentCommand);
		boost::unique_lock<boost::mutex> lock3(lockPositionsQueue);
		while (!positionsQueue.empty()){
			positionsQueue.pop();
		}
		for (int i=1;i<targetCount;i++) {
			positionsQueue.push(targets[i]);
		}
	}
	setJointPos(targets[0]);
	ROS_INFO("Joint trajectory started.");

}

void Kuka::Server::setMessage(const char *msg) {
	strcpy(currentCommand.message, msg);
}

IncomingDataPacket Kuka::Server::getCurrentState() {
	IncomingDataPacket d;
	boost::unique_lock<boost::mutex> lock(lockCurrentState);
	memcpy(&d,&currentState,sizeof(IncomingDataPacket));
	return d;
}

void Kuka::Server::softStop(){
	boost::unique_lock<boost::mutex> lock(lockPositionsQueue);
	while (!positionsQueue.empty()){
		positionsQueue.pop();
	}
	boost::unique_lock<boost::mutex> lock2(lockCurrentCommand);
	currentCommand.driveMode=STOPPED;
}


void Kuka::Server::requestShutdown(){
	shutdown_req=true;
}


bool Kuka::Server::haveTargetPoints(){
	return (currentCommand.driveMode!=STOPPED);
}

int Kuka::Server::numberOfWaypoints(){
	return positionsQueue.size();
}

ros::Time Kuka::Server::getRosTimeFromIPOC(long long int ipoc){
	long long int diff = ipoc - time_zero.first;
	long long int ms_diff = diff * CYCLE_TIME_MS;
	long long int ms=ms_diff%1000;
	return time_zero.second = ros::Time((ms_diff-ms)/1000,ms*1000000);
}



bool Server::fillOutgoingBuffer() {
	boost::unique_lock<boost::mutex> lock(lockCurrentCommand);
	boost::unique_lock<boost::mutex> lock2(lockPositionsQueue);

	// safe to look at current state as in same thread, no need to lock it..
	if (( currentCommand.target_ipoc <= currentState.ipoc ) && (currentCommand.driveMode!=STOPPED)) {
		ROS_DEBUG("Trajectory target point reached. ");
		ROS_DEBUG_STREAM("Target: " << currentCommand.axisTarget.a1 << ", "<<
				currentCommand.axisTarget.a2 << ", " <<
				currentCommand.axisTarget.a3 << ", " <<
				currentCommand.axisTarget.a4 << ", "<<
				currentCommand.axisTarget.a5 << ", "<<
				currentCommand.axisTarget.a6) ;
		ROS_DEBUG_STREAM("Actual: " << currentState.actualAxisPosition.axis[0] << ", "<<
				currentState.actualAxisPosition.axis[1] << ", " <<
				currentState.actualAxisPosition.axis[2] << ", " <<
				currentState.actualAxisPosition.axis[3] << ", "<<
				currentState.actualAxisPosition.axis[4] << ", "<<
				currentState.actualAxisPosition.axis[5]) ;
		// Either pull in next queued joint position or change to stopped state.
		if (positionsQueue.size()>0){
			while ((positionsQueue.size()>0 ) && (currentCommand.target_ipoc <= currentState.ipoc)) {
				lock.unlock();
				setJointPos(positionsQueue.front());
				lock.lock();
				positionsQueue.pop();
			}
		}
	}
	if ((currentCommand.target_ipoc <= currentState.ipoc) && (currentCommand.driveMode!=STOPPED)) {
		currentCommand.driveMode=STOPPED;
		ROS_INFO("No more trajectory points. STOPPED.");
	}

	// if safety problem
	//    zero it all.

	long long int ETA = currentCommand.target_ipoc - currentState.ipoc;

	switch (currentCommand.driveMode) {

	case JOINT_POS:
		double delta[6];
		// should be current state + previous correction...
//		delta[0] = currentCommand.axisTarget.a1 - currentState.actualAxisPosition.axis[0] ;
//		delta[1] = currentCommand.axisTarget.a2 - currentState.actualAxisPosition.axis[1] ;
//		delta[2] = currentCommand.axisTarget.a3 - currentState.actualAxisPosition.axis[2] ;
//		delta[3] = currentCommand.axisTarget.a4 - currentState.actualAxisPosition.axis[3] ;
//		delta[4] = currentCommand.axisTarget.a5 - currentState.actualAxisPosition.axis[4] ;
//		delta[5] = currentCommand.axisTarget.a6 - currentState.actualAxisPosition.axis[5] ;

		delta[0] = currentCommand.axisTarget.a1 - internal_state[0] ;
		delta[1] = currentCommand.axisTarget.a2 - internal_state[1] ;
		delta[2] = currentCommand.axisTarget.a3 - internal_state[2] ;
		delta[3] = currentCommand.axisTarget.a4 - internal_state[3] ;
		delta[4] = currentCommand.axisTarget.a5 - internal_state[4] ;
		delta[5] = currentCommand.axisTarget.a6 - internal_state[5] ;

		correction[0]=delta[0] / ETA;
		correction[1]=delta[1] / ETA;
		correction[2]=delta[2] / ETA;
		correction[3]=delta[3] / ETA;
		correction[4]=delta[4] / ETA;
		correction[5]=delta[5] / ETA;
		internal_state[0]+=correction[0];
		internal_state[1]+=correction[1];
		internal_state[2]+=correction[2];
		internal_state[3]+=correction[3];
		internal_state[4]+=correction[4];
		internal_state[5]+=correction[5];
		ROS_DEBUG_STREAM("Pos:"<<currentState.actualAxisPosition.axis[0]<<" "<<currentState.actualAxisPosition.axis[1]<<" "<<currentState.actualAxisPosition.axis[2]<<" "<<currentState.actualAxisPosition.axis[3]<<" "<<currentState.actualAxisPosition.axis[4]<<" "<<currentState.actualAxisPosition.axis[5]);
		ROS_DEBUG_STREAM("Intern:"<<internal_state[0]<<" "<<internal_state[1]<<" "<<internal_state[2]<<" "<<internal_state[3]<<" "<<internal_state[4]<<" "<<internal_state[5]);
		ROS_DEBUG_STREAM("Corr:"<<correction[0]<<" "<<correction[1]<<" "<<correction[2]<<" "<<correction[3]<<" "<<correction[4]<<" "<<correction[5]);
		sprintf(outgoingDataBuffer, outgoingXMLFormat,
				currentCommand.message,
				0.0, 0.0, 0.0,  0.0, 0.0, 0.0, // zero cartesian corrections...
				correction[0] ,
				correction[1],
				correction[2],
				correction[3],
				correction[4],
				correction[5],
				currentState.ipoc
				);
		break;
	case JOINT_VEL:
	case CART_VEL:
	case STOPPED:
//		std::cout << "Blocked.\n";
		sprintf(outgoingDataBuffer, outgoingXMLFormat,
				currentCommand.message,
				0.0, 0.0, 0.0,  0.0, 0.0, 0.0, // zero cartesian corrections...
				0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
				currentState.ipoc
				);
		break;
	}

	return true;

}
