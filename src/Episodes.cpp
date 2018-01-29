#include "Episodes.h"
#include <iostream>
using namespace std;

Episodes::Episodes()
{
	reset();
}

void Episodes::append(Event e)
{
	e.event_id = current_event_id;
	//e.episode_id = current_episode_id;
	data.push_back(e);

	current_event_id++;
}

void Episodes::reset(void)
{
	data.clear();
	current_event_id = 0;
}

/*
void Episodes::renewEpisode(void)
{
	current_event_id = 0;
	current_episode_id++;
}
*/

void Episodes::print(string filename)
{
	ofstream ofs;
	ostream *io;
	if(filename == "")
		io = &cout;
	else{
		ofs.open(filename);
		io = &ofs;
	}
		
		
	for(auto e : data){
		*io << e.str() << endl;
	}
}

Event *Episodes::At(int pos)
{
	if(pos < 0 || pos >= data.size())
		return NULL;
	return &(data[pos]);
}

Observation *Episodes::obsAt(int pos)
{
	if(pos < 0 || pos >= data.size())
		return NULL;
	return &(data[pos].observation);
}

Action *Episodes::actionAt(int pos)
{
	if(pos < 0 || pos >= data.size()){
		cerr << "OUT OF INDEX at actionAt" << endl;
		exit(1);
		return NULL;
	}
	return &(data[pos].action);
}
