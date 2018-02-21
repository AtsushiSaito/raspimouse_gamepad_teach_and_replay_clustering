#ifndef __PF_H__
#define __PF_H__

#include <vector>
#include "Particle.h"
#include "ProbDistribution.h"
#include "raspimouse_gamepad_teach_and_replay_clustering/PFoEOutput.h"
using namespace ros;

class Episodes;
class Observation;
class Action;

class ParticleFilter
{
public:
	ParticleFilter(int num, Episodes *ep);
	void init(void);
	void print(void);

//	Action sensorUpdate(Observation *obs, Episodes *ep, raspimouse_gamepad_teach_and_replay_clustering::PFoEOutput *out);
	Action sensorUpdate(Observation *obs, Action *act, Episodes *ep, raspimouse_gamepad_teach_and_replay_clustering::PFoEOutput *out);
	Action mode(Episodes *ep);
	Action modeParticle(Episodes *ep);
	Action average(Episodes *ep);
	void motionUpdate(Episodes *ep);
	void setClustering(vector<vector<double> > &p_prob, vector<vector<int> > &p);
private:
	int mode_event_id, predict_time, recent_predict, end_time;
	bool end_mode;
	vector<vector<double> > predict_proba;
	vector<vector<int> > predict;
	vector<Particle> particles;
	ProbDistributions prob;

	Episodes *episodes;

	double likelihood(Observation *past, Observation *last);
	double likelihood(Observation *past, Observation *last, Action *past_a, Action *last_a);

	void resampling(vector<Particle> *ps);
	void normalize(void);
};

#endif
