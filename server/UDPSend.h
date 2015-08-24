/*
 *  UDPSend.h
 *  MySender
 *
 *  Created by Helmut Hlavacs on 10.12.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

//#include <winsock2.h>
//#include <winsock.h>
#include <windows.h>

#pragma comment(lib, "wsock32.lib")	//WS2_32

/*
extern "C" {
//#include <stdlib.h>
//#include <unistd.h>
//#include <io.h>
//#include <netdb.h>
//#include <winsock.h>
#include <winsock2.h>
#include <windows.h>
//#include <sys/types.h>
/*#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>*/
//#include <string.h>
//}*/



class UDPSend {

public:
	//int sock;
	SOCKET sock;
	struct sockaddr_in addr;
	unsigned long packetnum;

	UDPSend();
	~UDPSend(){ if(sock != INVALID_SOCKET) closesocket(sock); };
	void init( char *address, int port );
	int send( char *buffer, int len  );
	void closeSock();
	int startWinsock(void);
	//SOCKET mysocket;
};



