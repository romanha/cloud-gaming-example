

#define XVID_CSP_RGB      (1<<16) /* 24-bit rgb packed */

#include "DecodeVideoXVID.h"


DecodeVideoXVID::DecodeVideoXVID(void) {

}

DecodeVideoXVID::~DecodeVideoXVID(void) {

}


int DecodeVideoXVID::dec_init(int width, int height) {
	
	xvid_dec_create_t xvid_dec_create;
	memset(&xvid_dec_create, 0, sizeof(xvid_dec_create_t));
	xvid_dec_create.version = XVID_VERSION; /* Version */
	/*
	 * Image dimensions -- set to 0, xvidcore will resize when ever it is
	 * needed
	 */
	
	xvid_dec_create.width = width;
	xvid_dec_create.height = height;
	
	int ret = xvid_decore(NULL, XVID_DEC_CREATE, &xvid_dec_create, NULL);
	dec_handle = xvid_dec_create.handle; //void*
	
	return(ret);
}

int DecodeVideoXVID::dec_main(void* istream, int istream_size, void* outbuffer, int width) {

	xvid_dec_frame_t xvid_dec_frame;
	
	xvid_dec_stats_t xvid_dec_stats;
	
	memset(&xvid_dec_frame, 0, sizeof(xvid_dec_frame_t)); /* Resetall structures*/
	
	xvid_dec_frame.version= XVID_VERSION; /* Set version*/
	
	/* No general flags to set */
	xvid_dec_frame.general = 0;

	/* Input stream */
	xvid_dec_frame.bitstream = istream;
	xvid_dec_frame.length = istream_size;
	
	/* Output framestructure */
	xvid_dec_frame.output.plane[0] = outbuffer;
	xvid_dec_frame.output.stride[0] = width * 3;
	xvid_dec_frame.output.csp = XVID_CSP_RGB;
	
	//int ret = xvid_decore(dec_handle, XVID_DEC_DECODE, &xvid_dec_frame, &xvid_dec_stats);
	int ret = xvid_decore(dec_handle, XVID_DEC_DECODE, &xvid_dec_frame, NULL);

	return(ret);
}