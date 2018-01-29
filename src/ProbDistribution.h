#ifndef __PROB_DISTRIBUTIONS
#define __PROB_DISTRIBUTIONS

#include <random>
using namespace std;

class ProbDistributions
{
public:
	ProbDistributions()
	{
		rd = new random_device();
		gen = new mt19937((*rd)());
	}

	~ProbDistributions()
	{
		delete gen;
		delete rd;
	}

	double normalRand(double mean, double stddev)
	{
		normal_distribution<> nd(mean,stddev);
		return nd(*gen);
	}

	double uniformRand(double min, double max)
	{
		uniform_real_distribution<> ud(min,max);
		return ud(*gen);
	}

	int uniformRandInt(int min,int max)
	{
		uniform_int_distribution<> ud(min,max);
		return ud(*gen);
	}

private:
	random_device *rd;
	mt19937 *gen;
};

#endif
