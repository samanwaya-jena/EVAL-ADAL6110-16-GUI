#ifndef AP_H
#define AP_H

namespace awl
{

struct APPoint {
	int x;
	int y;
};

struct APRect {
	int height;
	int width;
	int x;
	int y;
};

struct APFrame {
	int size;
	int cols;
	int rows;
	unsigned char * data;
};
}

#endif // AP_H
