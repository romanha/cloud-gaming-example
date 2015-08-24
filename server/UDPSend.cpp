/*
 *  UDPSend.cpp
 *  MySender
 *
 *  Created by Helmut Hlavacs on 10.12.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */


#include "UDPSend.h"



extern "C" {
#include <stdio.h>
#include <time.h>
}

#define min(x,y)  ((x) <= (y) ? (x) : (y))
#define max(x,y)  ((x) >= (y) ? (x) : (y))


typedef struct RTHeader {
	double		  time;
	unsigned long packetnum;
	unsigned char fragments;
	unsigned char fragnum;
} RTHeader_t;


UDPSend::UDPSend() {
	packetnum=0;
	sock=0;
}


int UDPSend::startWinsock(void)
{
	WSADATA wsa;
	return WSAStartup(MAKEWORD(2,0),&wsa);
}



void UDPSend::init(char *address, int port ) {
	if( sock ) closesocket(sock);
	
	sock = socket( PF_INET, SOCK_DGRAM, 0 );

	addr.sin_family = AF_INET;  
	addr.sin_addr.s_addr = inet_addr(address);
	addr.sin_port = htons( port );
}


int UDPSend::send( char *buffer, int len ) {
	char sendbuffer[65000];
	
	packetnum++;
	
	if( len>65000 ) {
		return 0;
	}
	//if( packetnum%100==50) return -1;		//test for packet loss
	
    int maxpacketsize = 1400;
	
	RTHeader_t header;
	header.time = clock() / (double)CLOCKS_PER_SEC;
	header.packetnum = packetnum;
	header.fragments = len/maxpacketsize;
	header.fragnum=1;
	
	int left = len - header.fragments * maxpacketsize;
	if( left>0 ) header.fragments++;
	
	int ret, bytes=0;
	for( int i=0; i<header.fragments; i++ ) {
		memcpy( sendbuffer, &header, sizeof( header ) );
        memcpy( sendbuffer + sizeof( header), buffer+bytes, min(maxpacketsize, len - bytes) );
		
		ret = sendto( sock, sendbuffer, min(maxpacketsize, len - bytes) + sizeof(header), 0, (const struct sockaddr *) &addr, sizeof(addr) );

        if(ret == -1 ) {
			return ret;
		} else {
			ret = ret - sizeof( header );
		}
		bytes  += ret;
		header.fragnum++;
	}
	
	return bytes;
}


void UDPSend::closeSock() {
	closesocket( sock );
	sock=0;
}








