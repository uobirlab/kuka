/*
 * CommsSim.cpp
 *
 *  Created on: 5 Dec 2011
 *      Author: burbrcjc
 *
 *  Quick and dirty Kuka "simulator" connects to the device driver and pretends to be the real Kuka
 *  arm. Sends information every 12ms, receives corrections, etc.
 *
 *  Does not do asynchronous comms, so not like Kuka...
 *
 *  
 *    
 */

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <sys/time.h>

#include <unistd.h>

using boost::asio::ip::tcp;

enum {
	max_length = 1024
};

const char *outgoingXML =
"<Rob> \n\
<RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /> \n\
<RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /> \n\
<AIPos A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
<ASPos A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" A6=\"0.0\" /> \n\
<MACur A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" A6=\"0.0\" /> \n\
<Delay D=\"%d\" /> \n\
<IPOC>%d</IPOC> \n\
</Rob>\n\r";

struct OutgoingData {
	struct {
		double x,y,z,a,b,c;
	} actualCartPosition, setpointCartPosition ;
	struct {
		double a1,a2,a3,a4,a5,a6;
	} actualAxisPosition, setpointAxisPosition, motorCurrents;
	int delayedPackets; // How many packets have we missed/been late sending - more than 9 is a stop condition....
	long long int ipoc;

};

enum driveMode_t {CART_VEL,JOINT_VEL,JOINT_POS,STOPPED};

struct IncomingData {
	char message[300];
	struct {
		double a1,a2,a3,a4,a5,a6;
	} axisCorrection;

	// The ipoc cylce that we aim for this command to be completed or stopped by
	long long int ipoc_response;
};

template < class RT > static RT getTagVal( char *xml, const char *tagname, const char *closetag, RT (*atov)(const char*), char **pos) {
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

bool parseIncomingCommand(  char *xml, IncomingData& incoming ) {
	char *position;


	incoming.axisCorrection.a1 = getTagVal<double>(xml, "<AKorr A1=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a2 = getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a3 = getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a4 = getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a5 = getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a6 = getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);


	incoming.ipoc_response = getTagVal<long long int>(xml, "<IPOC>",  "</IPOC>",  &atoll, &position);

	char *start = strstr(xml,"<EStr>");
	char *end = strstr(start+6,"</EStr>");
	*end = '\0';
	strcpy(incoming.message, start+6);

	return true;
}

int main(int argc, char* argv[]) {
	FILE *f = fopen("log.txt","w");
	try {
	  std::string port,host;
		if (argc != 3) {
		  std::cout << "Usage: CommsSim <host> <port>\nWhere <host> is the PC running the Kuka driver and <port> is usually 6008\n Using defaults.\n";
		  port="6008";
		  host="localhost";
		} else {
		  port=argv[2];
		  host=argv[1];
		}

		boost::asio::io_service io_service;

		tcp::resolver resolver(io_service);
		tcp::resolver::query query(tcp::v4(), host,port);
		tcp::resolver::iterator iterator = resolver.resolve(query);
		boost::system::error_code socketErrorCode;


		tcp::socket s(io_service);
		s.connect(*iterator);

		char outgoingBuffer[10240];
		OutgoingData robotState;
		robotState.ipoc = 0;

		char incomingBuffer[10240];
		size_t incomingDataPos = 0;
		IncomingData incomingCommand;
		char message[300];

		using namespace std;
		timeval start_time;
		timeval now_time;
		timeval passed;

		while (true) {
			gettimeofday(&start_time, NULL);

			robotState.ipoc++;
//			cout << "IPOC: " << robotState.ipoc << endl;
			// Send state
			size_t outLength = sprintf(outgoingBuffer, outgoingXML,
					robotState.actualAxisPosition.a1,
					robotState.actualAxisPosition.a2,
					robotState.actualAxisPosition.a3,
					robotState.actualAxisPosition.a4,
					robotState.actualAxisPosition.a5,
					robotState.actualAxisPosition.a6,
					0,
					robotState.ipoc	);
//			cout << "Outgoing: " << outLength << endl;
			boost::asio::write(s, boost::asio::buffer(outgoingBuffer, outLength));


			char *location;
			do {
				size_t len = s.read_some( boost::asio::buffer(incomingBuffer + incomingDataPos, 10240 - incomingDataPos), socketErrorCode);
				incomingDataPos += len;
				incomingBuffer[incomingDataPos+1] = '\0'; // stop the string search at the end
				location = strstr((char*)&(incomingBuffer[0]), "</Sen>");

			} while (location==NULL); // reached end of robot xml blip

			if (location + 8 - incomingBuffer == incomingDataPos) {
				//	safe to return to buffer beginning....
				incomingDataPos = 0;
			}
			parseIncomingCommand(incomingBuffer,incomingCommand);

			fprintf(f,"%f\t%f\t%f\t%f\t%f\%f\n",incomingCommand.axisCorrection.a1,
					incomingCommand.axisCorrection.a2,
					incomingCommand.axisCorrection.a3,
					incomingCommand.axisCorrection.a4,
					incomingCommand.axisCorrection.a5,
					incomingCommand.axisCorrection.a6 );

			// Apply whatever correction is to the state...no proper simulation....
			robotState.actualAxisPosition.a1+=incomingCommand.axisCorrection.a1;
			robotState.actualAxisPosition.a2+=incomingCommand.axisCorrection.a2;
			robotState.actualAxisPosition.a3+=incomingCommand.axisCorrection.a3;
			robotState.actualAxisPosition.a4+=incomingCommand.axisCorrection.a4;
			robotState.actualAxisPosition.a5+=incomingCommand.axisCorrection.a5;
			robotState.actualAxisPosition.a6+=incomingCommand.axisCorrection.a6;

			// If there is a message display it
			if (strcmp(message, incomingCommand.message) != 0 ) {
				strcpy(message, incomingCommand.message);
				std::cout <<incomingCommand.ipoc_response <<": " << incomingCommand.message << endl;
			}


			if (robotState.ipoc % 1000 == 0)
				cout <<incomingCommand.ipoc_response  << endl;

			usleep(12000);
			// bool delay = true;
			// while(delay){
			// 	gettimeofday(&now_time, NULL);
			// 	timersub(&now_time, &start_time, &passed);
			// 	if (passed.tv_usec > 12000)
			// 		delay=false;
			// }


		}
	} catch (std::exception& e) {
		std::cerr << "Bollox: " << e.what() << "\n";
	}
	fclose(f);

	return 0;
}
