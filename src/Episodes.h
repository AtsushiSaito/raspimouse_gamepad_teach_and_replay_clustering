#ifndef __EPISODE_H__
#define __EPISODE_H__

#include "ros/ros.h"
#include "Event.h"
#include <string>

class Episodes
{
public:
	Episodes();
	vector<Event> data;

	void append(Event e);
	void print(string filename = "");
	//void renewEpisode(void);
	void reset(void);

	Event *At(int pos);
	Observation *obsAt(int pos);
	Action *actionAt(int pos);
private:
//	int current_episode_id;
	int current_event_id;
};

#endif
