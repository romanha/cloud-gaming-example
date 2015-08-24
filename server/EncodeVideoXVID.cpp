

#define XVID_CSP_RGB      (1<<16) /* 24-bit rgb packed */

#include "EncodeVideoXVID.h"


EncodeVideoXVID::EncodeVideoXVID(void) {

}

EncodeVideoXVID::~EncodeVideoXVID(void) {

}

void EncodeVideoXVID::global_init(bool use_assembler) {

	xvid_gbl_init_t xvid_gbl_init;
	memset(&xvid_gbl_init, 0, sizeof(xvid_gbl_init_t)); /* Reset the structure with zeros */
	
	xvid_gbl_init.version = XVID_VERSION; /* Version */

	/* Assembly setting */
	if(use_assembler)
		#ifdef ARCH_IS_IA64
			xvid_gbl_init.cpu_flags = XVID_CPU_FORCE | XVID_CPU_IA64;
		#else
			xvid_gbl_init.cpu_flags = 0;
		#endif
	else
		xvid_gbl_init.cpu_flags = XVID_CPU_FORCE;
	
	xvid_global(NULL, 0, &xvid_gbl_init, NULL);
}

int EncodeVideoXVID::enc_init(int width, int height, int bitrate_target, int frinc, int frbase, int maxquant) {

	xvid_enc_create_t xvid_enc_create;
	xvid_plugin_single_t single; 
	
	//single pass rate control plgin
	xvid_enc_plugin_t plugins[8]; 

	memset(&xvid_enc_create, 0, sizeof(xvid_enc_create));
	xvid_enc_create.version = XVID_VERSION;
	xvid_enc_create.profile = 0xf4; 
	xvid_enc_create.width = width;
	xvid_enc_create.height = height;

	xvid_enc_create.plugins = plugins;
	xvid_enc_create.num_plugins = 0;
	
	memset(&single, 0, sizeof(xvid_plugin_single_t));
	single.version = XVID_VERSION;
	single.bitrate = bitrate_target;
	single.reaction_delay_factor = 16;
	single.averaging_period = 50;
	single.buffer = 50;

	plugins[xvid_enc_create.num_plugins].func = xvid_plugin_single;
	plugins[xvid_enc_create.num_plugins].param = &single;
	xvid_enc_create.num_plugins++;
	xvid_enc_create.num_threads = 1;	//= ARG_THREADS
	xvid_enc_create.max_bframes = 0;
	xvid_enc_create.fincr = frinc;
	xvid_enc_create.fbase = frbase;
	xvid_enc_create.max_key_interval = 30;
	xvid_enc_create.frame_drop_ratio = 0;
	xvid_enc_create.min_quant[0] = 1;
	xvid_enc_create.min_quant[1] = 1;
	xvid_enc_create.min_quant[2] = 1;
	xvid_enc_create.max_quant[0] = maxquant; //>=0
	xvid_enc_create.max_quant[1] = maxquant;
	xvid_enc_create.max_quant[2] = maxquant;
	int xerr = xvid_encore(NULL, XVID_ENC_CREATE, &xvid_enc_create, NULL);
	
	/* Retrieve the encoder instance from the structure */
	enc_handle = xvid_enc_create.handle; //void*
	
	return (xerr);

}

int EncodeVideoXVID::enc_main(void *image,  void *bitstream, int width, int height) {
	xvid_enc_frame_t xvid_enc_frame;
	xvid_enc_stats_t xvid_enc_stats;
	
	/* Version for the frame and the stats */
	memset(&xvid_enc_frame, 0, sizeof(xvid_enc_frame));
	xvid_enc_frame.version = XVID_VERSION;
	memset(&xvid_enc_stats, 0, sizeof(xvid_enc_stats));
	xvid_enc_stats.version = XVID_VERSION;
	
	/* Bind output buffer */
	xvid_enc_frame.bitstream = bitstream;
	xvid_enc_frame.length = width * height * 3;
	xvid_enc_frame.input.csp = XVID_CSP_RGB;
	xvid_enc_frame.input.plane[0] = image;
	xvid_enc_frame.input.stride[0] = 3 * width; //bytes per row
	
	//.... /* Set up core's general features */
	
	/* Frame type -- taken from function call parameter */
	xvid_enc_frame.type = XVID_TYPE_AUTO;
	//xvid_enc_frame.type = XVID_TYPE_IVOP;
	
	/* Force the right quantizer -- It is internally managed by RC plugins */
	xvid_enc_frame.quant = -1; //could try to adapt quantization manually here
	
	/* Encode the frame */
	int ret = xvid_encore(enc_handle, XVID_ENC_ENCODE, &xvid_enc_frame, &xvid_enc_stats);
	int key = (xvid_enc_frame.out_flags & XVID_KEYFRAME); //key frame?
	
	return (ret);
}