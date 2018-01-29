#include "Event.h"
#include "Observation.h"
#include <iostream>
#include <string>

Event::Event(Observation obs,Action act, int rw)
{
	reward = rw;
	action = act;
	observation = obs;
	counter = 0;
}

string Event::str(void)
{
//	string id = to_string(episode_id) + '\t' + to_string(event_id);
	string id = to_string(event_id);
	string a = to_string(action.linear_x) + '\t'
		+ to_string(action.angular_z);
	string s = to_string(observation.lf) + '\t'
		+ to_string(observation.ls) + '\t' 
		+ to_string(observation.rs) + '\t' 
		+ to_string(observation.rf);

	string t = to_string(time.toSec());

	return id + '\t' + t + '\t' + a + '\t' + s + " 0";
}

