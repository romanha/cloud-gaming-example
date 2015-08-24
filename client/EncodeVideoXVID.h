extern "C" {
	#include <xvid.h>
}

#include <stdio.h>
#include <string.h>


class EncodeVideoXVID
{
public:
	EncodeVideoXVID(void);
	virtual ~EncodeVideoXVID(void);
 
	void global_init(bool use_assembler);
	int enc_init(int width, int height, int bitrate_target, int frinc, int frbase, int maxquant);
	int enc_main(void *image, void *bitstream, int width, int height);

protected:
	void* enc_handle;
};