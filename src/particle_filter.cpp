/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang, additional edits made by Mwesigwa Musisi-Nkambwe for completion of assignment (Dec 2021)
 * https://githubhelp.com/castramet/uWebSockets - used for websockets in main
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <iostream>


#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
  
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 200;  // TODO: Set the number of particles
  


  // Create sensor noise normal distribution for x, y and theta using std::normal_distribution (mean, standard deviation)
  
  std::normal_distribution<double> N_x(x, std[0]);
  std::normal_distribution<double> N_y(y, std[1]);
  std::normal_distribution<double> N_theta(theta, std[2]);
  
  for (int i = 0; i < num_particles; ++i) 
  {  
   
    Particle newParticle;
    
   // Initialization of particles.
    newParticle.id = i;
    newParticle.x = x;
    newParticle.y = y;
    newParticle.theta = theta;
    newParticle.weight = 1.0;
    
    // Adding noise
    newParticle.x += N_x(gen);
    newParticle.y += N_y(gen);
    newParticle.theta += N_theta(gen);
    
    // Add a new element at the end of the vector, after its current last element. 
    particles.push_back(newParticle);
    weights.push_back(newParticle.weight); 
    
  }
is_initialized = true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) 
{
  
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  //Create normal distributions for x, y and theta
  std::normal_distribution<double> N_x(0, std_pos[0]);
  std::normal_distribution<double> N_y(0, std_pos[1]);
  std::normal_distribution<double> N_theta(0, std_pos[2]);
  
  for (int i = 0; i < num_particles; i++) {

    // calculate new state
    if (fabs(yaw_rate) < 0.00001) 
    {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else 
    {
      particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Add noise
    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) 

{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (unsigned int i = 0; i < observations.size(); i++) 
  {
    
    // Get current observation
    LandmarkObs observe = observations[i];

    // initialize minimum distance to a largest possible double value for later comparison
    double minimum_distance = std::numeric_limits<double>::max();

    // initialize landmark identifier from map placeholder associated with the observation
    int map_identifier;
    
    for (unsigned int j = 0; j < predicted.size(); j++) 
    {
      
      // Get distance between current & predicted landmarks
      double current_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      // Look for predicted landmark nearest the current observed landmark
      // Here I am starting with the largest possible double number them decreasing each iteration 
      if (current_distance < minimum_distance) 
      {
        minimum_distance = current_distance;
        map_identifier = predicted[j].id;
      }
    }
    observations[i].id = map_identifier;
  }
}
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) 
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   *   Steps taken are:
   *   Step1. TRANSFORM each observation marker from the vehicle's coordinates to the map's coordinates
   *   Step2. ENSURE map landmarks are inside sensor range
   *   Step3. Nearest Neighbor Data Association
   *   Step 4. Compute weight of particle
   */
  // For each Particle

  for (int i = 0; i < num_particles; i++)
  {
    Particle p = particles[i];
    // TRANSFORM each observation marker from the vehicle's coordinates to the map's coordinates

    vector<LandmarkObs> transformed_observations; 
    double t_x,t_y;
    
    
    for(unsigned int j = 0; j < observations.size(); j++) 
    {
      LandmarkObs obs = observations[j];
      t_x = observations[j].x*cos(p.theta) - observations[j].y*sin(p.theta) + p.x;
      t_y = observations[j].y*cos(p.theta)+ observations[j].x*sin(p.theta) + p.y;
      transformed_observations.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
    }
    // Each new map landmark within sensor's range is put in a list of landmarks. 

    vector<LandmarkObs> predictions;

    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) 
    {
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
	  int landmark_id = map_landmarks.landmark_list[j].id_i;
      if(fabs(t_x - p.x) <= sensor_range && fabs(t_y - p.y) <= sensor_range) 
		{
			predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
		}
    }
    
    // Nearest Neighbor Data Association applied 

    dataAssociation(predictions, transformed_observations);
  // reinitializing weight
    particles[i].weight = 1.0;     // reset the particle's weight to 1 and recalculate

  
    LandmarkObs Land_A, obs;
    for (unsigned int j = 0; j < transformed_observations.size(); j++) 
	{
      obs = transformed_observations[j];
      // Pair Landmark with Associated id
      
      for (unsigned int k = 0; k < predictions.size(); k++) 
		{
        if (predictions[k].id == obs.id)
        {
          Land_A = predictions[k];
		  break;
		}
	}

      // calculate the weigh/probability of this particle using the multivariate Gaussian

	double mu_x = std_landmark[0];
	double mu_y = std_landmark[1];
    double exponent = pow((obs.x - Land_A.x), 2) / (2.0*mu_x*mu_x)  +  pow((obs.y - Land_A.y), 2) / (2.0*mu_y*mu_y);   

    particles[i].weight *= exp(-exponent) / (2.0 * M_PI * mu_x * mu_y);
    }
 	weights[i] = particles[i].weight;
}  
}
void ParticleFilter::resample() 
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<int> distribution (weights.begin(), weights.end());
  vector<Particle> resampled_particles;
  
  
  // create new weights variable with current weights
  vector<double> weights;
  
  for (unsigned int i = 0; i < num_particles; i++) 
  {
    weights.push_back(particles[i].weight);
  }
  
  for (unsigned int i=0; i < particles.size(); i++) 
  {
    int n = distribution(gen);
    resampled_particles.push_back(particles[n]);
  }  
  particles = resampled_particles;
}
  
void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) 
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) 
{
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
