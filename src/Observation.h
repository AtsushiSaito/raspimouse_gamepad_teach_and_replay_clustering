#ifndef __OBS_H_ 
#define __OBS_H_

#include <string>
#include <vector>
#include <fstream>
#include "ros/ros.h"
using namespace std;

class Observation{
public:
	Observation();
	Observation(int left_f,int left_s, int right_s, int right_f);

	void setValues(int left_f,int left_s, int right_s, int right_f);

	int lf;
	int ls;
	int rs;
	int rf;
	double log_lf;
	double log_ls;
	double log_rs;
	double log_rf;
};

#endif
