extern "C" {
	#include <xvid.h>
}

#include <stdio.h>
#include <string.h>


class DecodeVideoXVID
{
public:
	DecodeVideoXVID(void);
	virtual ~DecodeVideoXVID(void);

	int dec_init(int width, int height);
	int dec_main(void* istream, int istream_size, void* outbuffer, int width);

protected:
	void* dec_handle;
};