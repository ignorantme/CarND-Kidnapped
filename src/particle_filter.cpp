#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;
// A random seed
random_device r;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	num_particles = 100;
	weights.resize(num_particles);
	//Creates normal distributions for x, y and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	// A random engine to initialize the particles
	static default_random_engine gen(r());

	// Initialize particles
	for(int i = 0; i < num_particles; ++i){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	//  Add measurements to each particle and add random Gaussian noise.
	// A random engine to generate particles
	default_random_engine gen(r());

	//Create normal distributions for sensor noises
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// Predict the next state based on the given velocity and yaw_rate
	for(int i = 0; i < num_particles; i++){
		//if the yaw_rate == 0
		if(fabs(yaw_rate) < 1.0E-8){
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else{
			particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
			particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
			particles[i].theta += yaw_rate * delta_t;
		}
		 // Add sense noise to the particles
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//   Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.

	for(int i = 0; i < observations.size(); i++){
		LandmarkObs obs = observations[i];
		//initialize the minimum distance with infinity
		double min_dist = numeric_limits<double>::infinity();
		//the landmark id to me associate with the observation
		double obs_id = -1;
		//for each observations, find the nearest landmark
		for(int j = 0; j < predicted.size(); j ++){
			double dis = dist(obs.x, obs.y, predicted[j].x, predicted[j].y);
			if(dis < min_dist){
				min_dist = dis;
				obs_id = predicted[j].id;
			}
		}
		//set the observation's id to the nearest landmark
		observations[i].id = obs_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// for each particle
	for(int i = 0; i < num_particles; i++){
		vector<LandmarkObs> transformed;

		// for each observation, transform the the coordinates to map coordinate system
		for(int j = 0; j < observations.size(); j++){
			double obs_x = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
			double obs_y = particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;
			transformed.push_back(LandmarkObs{observations[j].id, obs_x, obs_y});

		}
		vector<Map::single_landmark_s> lm_list = map_landmarks.landmark_list;
		vector<LandmarkObs> predictions;
		for(int m = 0; m < lm_list.size(); m++){
				predictions.push_back(LandmarkObs{lm_list[m].id_i-1, lm_list[m].x_f, lm_list[m].y_f});
		}
		dataAssociation(predictions, transformed);

		//initialize the particle's weight
		particles[i].weight = 1.0;
		//get current particle's transformed sense data and associated landmarks
		vector<int> associations;
		vector<double>sense_x;
		vector<double>sense_y;
		//multiply the weights of each observations to get the final weight
		for(int j = 0; j < transformed.size(); j++){
			double obs_x = transformed[j].x;
			double obs_y = transformed[j].y;
			double obs_id = transformed[j].id;
			associations.push_back(obs_id+1);
			sense_x.push_back(obs_x);
			sense_y.push_back(obs_y);
			double pre_x, pre_y;

			for(int k = 0; k < map_landmarks.landmark_list.size(); k++){
				if(predictions[k].id == transformed[j].id){
					pre_x = predictions[k].x;
					pre_y = predictions[k].y;
				}
			}
			double w = exp(pow(pre_x-obs_x,2)/(-2*pow(std_landmark[0],2)) +
					pow(pre_y - obs_y, 2)/(-2*pow(std_landmark[1],2)))
					/ (2*M_PI*std_landmark[0]*std_landmark[1]);
			//product the weight of this observation with the total weight
			particles[i].weight *= w;

		}
		weights[i] = particles[i].weight;
		particles[i] = SetAssociations(particles[i],associations, sense_x,sense_y);
	}
}

void ParticleFilter::resample() {
	//  Resample particles with replacement with probability proportional to their weight.

	//a vector to put the resampled particles
	vector<Particle> new_particles;
	random_device rd;
	default_random_engine gen(rd());
	discrete_distribution<int> selecter(weights.begin(), weights.end());
	for(int i = 0; i < num_particles; i++){
		int index = selecter(gen);
		new_particles.push_back(particles[index]);
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
