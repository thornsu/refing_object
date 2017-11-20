#include"my_refind/config.h"

Config::Config(const string& camerapara)
{
	FileStorage fsSetting(camerapara, FileStorage::READ);
	if(!fsSetting.isOpened())
	{
		cerr<<"error:wrong path to setting"<<endl;
		throw;
	}
	cout<<"right path to setting"<<endl;
	
	fsSetting["LEFT.K"]>>K_l;
	fsSetting["RIGHT.K"]>>K_r;
	
	fsSetting["LEFT.D"]>>D_l;
	fsSetting["RIGHT.D"]>>D_r;
	
	fsSetting["LEFT2RIGHT.R"]>>L2R_R;
	fsSetting["LEFT2RIGHT.T"]>>L2R_T;
	
	
	rows_l=fsSetting["LEFT.height"];
	cols_l = fsSetting["LEFT.width"];
	
	rows_r=fsSetting["RIGHT.height"];
	cols_r = fsSetting["RIGHT.width"];

}

