#ifndef __EVENT__H_ 
#define __EVENT__H_ 

#include <string>
#include <vector>
#include <fstream>
#include "ros/ros.h"
#include "Observation.h"
using namespace std;

struct Action{double linear_x;double angular_z;};

class Event{
public:
	Event(Observation obs,Action act, int rw);
	int reward;
	Observation observation;
	Action action;
	ros::Time time;

	string str(void);

	int episode_id;
	int event_id;
	int counter;
};

#endif
