/*
 *  UDPReceive.cpp
 *  MyReceiver
 *
 *  Created by Helmut Hlavacs on 11.12.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

//#include "StdAfx.h"
#include "UDPReceive.h"

extern "C" {
	#include <stdio.h>
	#include <string.h>
}

typedef struct RTHeader {
	double		  time;
	unsigned long packetnum;
	unsigned char fragments;
	unsigned char fragnum;
} RTHeader_t;



UDPReceive::UDPReceive() {
	recbuffer = new char[65000];
}


void UDPReceive::init( int port ) {
	sock = socket( PF_INET, SOCK_DGRAM, 0 );
	
	addr.sin_family = AF_INET;  
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons( port );
	int ret = bind(sock, (const sockaddr *)&addr, sizeof(addr));
	
	printf("Binding port %d return %d\n", port, ret );
	
	leftover=false;
}


int UDPReceive::receive( char *buffer, int len, double *ptime ) {
	return receive( buffer, len, "", ptime );
}


int UDPReceive::receive( char *buffer, int len, char *tag, double *ptime ) {
	struct sockaddr_in si_other;
    int slen=sizeof(si_other);
	//socklen_t slen=sizeof(si_other);

	RTHeader_t *pheader = (RTHeader_t*)recbuffer;
	
	bool goon=true;
	int bytes = 0;
	int packetnum=-1;
	int fragments=-1;
	int fragnum=-1;
	int nextfrag=1;
	
	while( goon ) {
		int ret=0;
		
		if( !leftover ) {
			ret = recvfrom(sock, recbuffer, 65000, 0,(sockaddr*) &si_other, &slen);
		}
		leftover=false;
		
		//printf("%s UDP Packet %ld Size %d Fragment %d of %d Nextfrag %d\n", tag, pheader->packetnum, ret, pheader->fragnum, pheader->fragments, nextfrag );
		
		if( ret>sizeof( RTHeader_t ) ) {
			if( packetnum==-1 ) {						//first fragment of the new packet
				packetnum = pheader->packetnum;
			}
			
			if( packetnum != pheader->packetnum ) {		//last fragments lost
				printf("Last Frag %d lost", nextfrag );
				leftover = true;
				return -1;
			}

			//printf("%s UDP Packet %ld Size %d Fragment %d of %d Nextfrag %d\n", tag, pheader->packetnum, ret, pheader->fragnum, pheader->fragments, nextfrag );

			if( nextfrag!= pheader->fragnum ) {			//a fragment is missing
				printf("Fragment %d lost\n", nextfrag );
				return -1;
			}
			nextfrag++;
			
			memcpy( buffer+bytes, recbuffer + sizeof( RTHeader_t), ret - sizeof(RTHeader_t) );
			bytes += ret - sizeof( RTHeader_t );
			
			if(pheader->fragments == pheader->fragnum) goon=false;		//last fragment
			
			packetnum = pheader->packetnum;
			fragments = pheader->fragments;
			fragnum   = pheader->fragnum;
			
			*ptime = pheader->time;
			
		} else {
			printf("Fragment %d not larger than %d", pheader->fragnum, sizeof( RTHeader_t ) );
			return -1;
		}
		
	}
	
	leftover = false;
	return bytes;
}


int UDPReceive::startWinsock(void)
{
	WSADATA wsa;
	return WSAStartup(MAKEWORD(2,0),&wsa);
}


void UDPReceive::closeSock() {
	closesocket( sock );
}



