#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <cmath>
#include <winsock2.h>
#include <Windows.h>
#include <Ws2tcpip.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64 

#include "Leap.h"

using namespace Leap;

// http://stackoverflow.com/questions/10905892/equivalent-of-gettimeday-for-windows
// MSVC defines this in winsock2.h!?
/*typedef struct timeval {
    long tv_sec;
    long tv_usec;
} timeval;*/

int gettimeofday(struct timeval * tp, struct timezone * tzp);

float calculateTotalAngle(Vector *v, int size);
float calculateAngle(Vector v1, Vector v2);

#define PORT 32000
#define RADIANS_TO_DEGREES 57.2958

typedef int socklen_t;
typedef int ssize_t;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

int main(int argc, char* argv[])
{
	int status;
	int sockfd;
    struct sockaddr_in serv_addr;
    socklen_t serv_addr_len = sizeof(serv_addr);
	struct timeval tv1, tv2;
	Controller controller;
	char sendbuf[256];
	int totalangle = 0;
	int turn = 0;

    std::cout << "Enter the target IP address: ";
    char address_ascii[128];
    std::cin >> address_ascii;

    std::cout << "Enter the target port: ";
    unsigned short port;
    std::cin >> port;

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, address_ascii, &(serv_addr.sin_addr)) == 0)
    {
        std::cerr << "Call to InetPton failed." << std::endl;
        exit(3);
    }

	// Initializes wsock32
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		std::cerr << "Call to WSAStartup failed.\n" << std::endl;
		exit(1);
	}

	// Creates a socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        std::cerr << "Call to socket failed." << std::endl;
        exit(2);
    }

	std::cout << "Calling connect" << std::endl;
    if (connect(sockfd, (struct sockaddr*)&serv_addr, serv_addr_len) == -1)
    {
		closesocket(sockfd);
        std::cerr << "Call to accept failed." << std::endl;
        exit(3);
    }

    std::cout << "Connected to " << inet_ntoa(serv_addr.sin_addr) << ":" << (int)ntohs(serv_addr.sin_port) << std::endl;

	gettimeofday(&tv1, NULL);

	while (1)
	{
		gettimeofday(&tv2, NULL);
		
		if (tv2.tv_sec - tv1.tv_sec >= 5)
		{
			Frame frame = controller.frame();
			HandList hands = frame.hands();
			for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl)
			{
				// Get the first hand
				const Hand hand = *hl;

				const FingerList indexFingerList = hand.fingers().fingerType(Finger::TYPE_INDEX);
				const Finger indexFinger = indexFingerList[0];

				Vector v[4];
				// Get finger bones
				for (int b = 0; b < 4; ++b)
				{
					Bone::Type boneType = static_cast<Bone::Type>(b);
					Bone bone = indexFinger.bone(boneType);
					/*std::cout << std::string(6, ' ') << boneNames[boneType]
						<< " bone, direction: " << bone.direction() << std::endl;*/
					v[b] = bone.direction();
				}
				totalangle = (int)(RADIANS_TO_DEGREES*calculateTotalAngle(v, 4));
				std::cout << "Total angle = " << totalangle << std::endl;
				std::cout << std::endl;

			}
			memset(sendbuf, 0, sizeof(sendbuf));
			if (totalangle > 130)
			{
				turn = 1;
			}
			itoa(turn, sendbuf, 10);
			std::cout << "Turn = " << turn << std::endl;
			std::cout << "Calling send" << std::endl;
			if (send(sockfd, sendbuf, sizeof(sendbuf), 0) == -1)
			{
				closesocket(sockfd);
				std::cerr << "Call to send failed.\n" << std::endl;
				exit(1);
			}
			std::cout << "Return from send" << std::endl;
			turn = 0;
			/*if ((++totalangle) % 100 == 0)
			{
				totalangle = 0;
			}*/

			gettimeofday(&tv1, NULL);
		}
	}

	// Closes socket
	closesocket(sockfd);
	WSACleanup();

	return 0;
}

int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
    // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL);

    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;

    GetSystemTime( &system_time );
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime )      ;
    time += ((uint64_t)file_time.dwHighDateTime) << 32;

    tp->tv_sec  = (long) ((time - EPOCH) / 10000000L);
    tp->tv_usec = (long) (system_time.wMilliseconds * 1000);
    return 0;
}

float calculateTotalAngle(Vector *v, int size)
{
	float angle = 0;

	for (int i = 0; i < size-1; i++)
	{
		float temp = v[i].angleTo(v[i + 1]);
		angle += temp;
		std::cout << "Angle " << i << " = " << temp << std::endl;
	}

	return angle;
}

float calculateAngle(Vector v1, Vector v2)
{
	return acos(v1.dot(v2)/(v1.magnitude()*v2.magnitude()));
}
