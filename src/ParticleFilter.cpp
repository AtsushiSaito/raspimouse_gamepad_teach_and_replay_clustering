#include <vector>
#include "Particle.h"
#include "ParticleFilter.h"
#include "Observation.h"
#include "Episodes.h"
using namespace std;

ParticleFilter::ParticleFilter(int num, Episodes *ep)
{
	double w = 1.0/num;
	Particle p(w);
	for(int i=0;i<num;i++){
		particles.push_back(p);
	}
	episodes = ep;
}

void ParticleFilter::init(void)
{
	double w = 1.0/particles.size();
	for(auto &p : particles){
		p.pos = prob.uniformRandInt(0,episodes->data.size()-2);
		p.weight = w;
	}
	end_mode = false;
	end_time = 0;
}

void ParticleFilter::print(void)
{
	int i = 0;
	for(auto &p : particles){
		if(p.pos >= 0){
			auto d = episodes->data[p.pos];
			cout << d.str() << endl;
			i++;
		}
		if(i==10)
			return;
	}
}

Action ParticleFilter::modeParticle(Episodes *ep)
{
	double fw = 0.0;
	double rot = 0.0;
	double max = 0.0;
	cout << "mode particle" << endl;
	for(auto &p : particles){
		auto e = ep->actionAt(p.pos);
		if(max < p.weight){
			max = p.weight;
			fw = e->linear_x;
			rot = e->angular_z;
		}
	}
	Action a;
	a.linear_x = fw;
	a.angular_z = rot;
	return a;
}

Action ParticleFilter::mode(Episodes *ep)
{
	for(auto &p : particles){
		auto e = ep->At(p.pos);
		e->counter = 0;
	}
	for(auto &p : particles){
		auto e = ep->At(p.pos);
		e->counter++;
	}
	int max = 0;
	Action *mode_a;
	for(auto &p : particles){
		auto e = ep->At(p.pos);
		if(e->counter > max){
			max = e->counter;
			mode_event_id = e->event_id;
			mode_a = ep->actionAt(p.pos+1);
		}
		e->counter = 0;
	}
	cout << "EventID: " << mode_event_id
	<< " ID: " << predict.at(mode_event_id).at(0)
	<< " label: " << predict.at(mode_event_id).at(1)
	<< " group: " << predict.at(mode_event_id).at(2)
	<< " time: " << predict.at(mode_event_id).at(3)
	<< " len: " << predict.at(mode_event_id).at(4)
	<< " fast: " << predict.at(mode_event_id).at(5)
	<< " last: " << predict.at(mode_event_id).at(6) << endl;

	Action a;
	a.linear_x = mode_a->linear_x;
	a.angular_z = mode_a->angular_z;
	return a;
}

Action ParticleFilter::average(Episodes *ep)
{
	double fw = 0.0;
	double rot = 0.0;
	cout << "avg" << endl;
	for(auto &p : particles){
		auto e = ep->actionAt(p.pos+1);
		fw += p.weight * e->linear_x;
		rot += p.weight * e->angular_z;
	}
	Action a;
	a.linear_x = fw;
	a.angular_z = rot;
	return a;
}

Action ParticleFilter::sensorUpdate(Observation *obs, Action *act, Episodes *ep, raspimouse_gamepad_teach_and_replay_clustering::PFoEOutput *out)
{
	out->eta = 0.0;
	cout << "obs likelihood" << endl;

	int e_predict = predict.at(mode_event_id).at(2);
	int e_len = predict.at(mode_event_id).at(4);
	int ep_max = episodes->data.size();

	if(not end_mode){
		cout << "predict_time: " << predict_time << endl;
		if(e_predict == recent_predict){
			if(predict_time >= e_len){
				e_predict++;
				if(e_predict > predict.at(ep_max - 1).at(2) - 1){
					end_mode = true;
					end_time = 0;
				}
			}else{
				predict_time++;
			}
		}else{
			predict_time = 0;
		}
		recent_predict = e_predict;
	}else if(end_mode){
		if(end_time > 10){
			end_mode = false;
			end_time = 0;
		}else{
			end_time++;
		}
	}

	for(auto &p : particles){
		double h = likelihood(episodes->obsAt(p.pos),obs);
		//double h = likelihood(episodes->obsAt(p.pos),obs, episodes->actionAt(p.pos), act);

		int p_predict = predict.at(episodes->At(p.pos)->event_id).at(2);

		bool mode = false;

		if(not mode && not end_mode){
			if(p_predict == e_predict)
				h *= 0.5;
			else if(p_predict == e_predict + 1)
				h *= 0.3;
			else
				h *= 0.2;
		}
		p.weight *= h;
		out->eta += p.weight;
	}
/*
	cout << "mode particle" << endl;
	double max = 0.0;
	Particle *max_p = NULL;
	for(auto &p : particles){
		if(max < p.weight){
			max = p.weight;
			max_p = &p;
		}
	}
	out->particles_pos.push_back(max_p->pos);
	return modeParticle(ep);
*/

	normalize();
	resampling(&particles);

	//cout << "OUTPUT " << fw << " " << rot << endl;

	for(auto &p : particles){
		out->particles_pos.push_back(p.pos);
	}

	cout << "mode" << endl;
	return mode(ep);
//	cout << "avg" << endl;
//	return average(ep);
}

double ParticleFilter::likelihood(Observation *past, Observation *last)
{
	double diff[4] = {	past->log_lf - last->log_lf,
				past->log_ls - last->log_ls,
				past->log_rs - last->log_rs,
				past->log_rf - last->log_rf };
	double ans = 1.0;
	for(double &d : diff){
		ans /= (1 + fabs(d));
	}

	return ans;
}

double ParticleFilter::likelihood(Observation *past, Observation *last, Action *past_a, Action *last_a)
{
	double diff[4] = {	past->log_lf - last->log_lf,
				past->log_ls - last->log_ls,
				past->log_rs - last->log_rs,
				past->log_rf - last->log_rf };

	double ans = 1.0;
	for(double &d : diff){
		ans /= (1 + fabs(d));
	}
	ans /= (1 + 0.2*fabs(past_a->linear_x - last_a->linear_x));
	ans /= (1 + 0.2*fabs(past_a->angular_z - last_a->angular_z));

	return ans;
}


void ParticleFilter::resampling(vector<Particle> *ps)
{
        vector<Particle> prev;
        std::shuffle(ps->begin(),ps->end(),std::mt19937());

        double suweighteight = 0.0;
        int num = (int)ps->size();
        for(int i = 0;i < num ;i++){
                ps->at(i).weight += suweighteight;
                suweighteight = ps->at(i).weight;
                prev.push_back(ps->at(i));
        }

        double step = suweighteight / num;
        int* choice = new int[num];
        //double accum = prob.uniformRand(0.0,1.0) / num;
        double accum = step * prob.uniformRand(0.0,0.999999999);
        int j = 0;
        for(int i=0;i<num;i++){
                while(prev[j].weight <= accum)
                        j++;

                if(j == num)
                        j--;

                accum += step;
                choice[i] = j;
        }

        for(int i=0;i<num;i++){
                int j = choice[i];
                ps->at(i) = prev[j];
                ps->at(i).weight = 1.0/num;
        }

        delete [] choice;
}

void ParticleFilter::normalize(void)
{
	double eta = 0.0;
	for(auto &p : particles)
		eta += p.weight;

	cout << "eta: " << eta << endl;
	for(auto &p : particles)
		p.weight /= eta;
}

void ParticleFilter::motionUpdate(Episodes *ep)
{
/*
	cout << "no odom" << endl;
	init();
*/
	cout << "odom" << endl;
	for(auto &p : particles){
		if(rand() % 10 == 0){
			p.pos = prob.uniformRandInt(0,episodes->data.size()-2);
			continue;
		}

		int r = rand() % 3;
		if(r==0)
			p.pos++;
		else if(r==1)
			p.pos += 2;

		if(p.pos >= ep->data.size()-1)
			p.pos = prob.uniformRandInt(0,episodes->data.size()-2);

	}
}

void ParticleFilter::setClustering(vector<vector<double> > &p_prob, vector<vector<int> > &p){
	predict_proba = p_prob;
	predict = p;
}
