#ifndef CONFIG_H
#define CONFIG_H

#include "my_refind/common.h"

class Config
{
public:
	Config(const string& camerapara);
	Mat K_l,K_r,D_l,D_r;
	Mat L2R_R,L2R_T;
	int rows_l,cols_l,rows_r,cols_r;
	
};

#endif 