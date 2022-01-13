/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang, additional edits made by Mwesigwa Musisi-Nkambwe for completion of assignement (Dec 2021)
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
  num_particles = 100;  // TODO: Set the number of particles
  


  // Create normal distributions for x, y and theta
  
  std::normal_distribution<double> N_x_init(0, std[0]);
  std::normal_distribution<double> N_y_init(0, std[1]);
  std::normal_distribution<double> N_theta_init(0, std[2]);
  
  for (int i = 0; i < num_particles; ++i) 
  {  
   
    Particle kidnap;
    
   // Initialization of particles.
    kidnap.id = i;
    kidnap.x = x;
    kidnap.y = y;
    kidnap.theta = theta;
    kidnap.weight = 1.0;
    
    // Adding noise
    kidnap.x += N_x_init(gen);
    kidnap.y += N_y_init(gen);
    kidnap.theta += N_theta_init(gen);
    
    // Add a new element at the end of the vector, after its current last element. 
    particles.push_back(kidnap);
    
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
    std::cout<<minimum_distance;

    // initialize landmark identifier from map placeholder associated with the observation
    int map_identifier;
    
    for (unsigned int j = 0; j < predicted.size(); j++) {
      // Get current prediction
      LandmarkObs prediction = predicted[j];
      
      // Get distance between current & predicted landmarks
      double current_distance = dist(observe.x, observe.y, prediction.x, prediction.y);

      // Look for predicted landmark nearest the current observed landmark
      // Here I am starting with the largest possible double number them decreasing each iteration 
      if (current_distance < minimum_distance) 
      {
        minimum_distance = current_distance;
        map_identifier = prediction.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
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
   */
  
  // Set standard deviations for x, y, and theta
  double std_x = 2;
  double std_y = 2;
  double std_theta = 0.05;
  
  // To keep track of each particle
   for (int i = 0; i < num_particles; i++) 
   {
     // x and y particle coordinates
     double x_part = particles[i].x;
     double y_part = particles[i].y;
     double theta_part = particles[i].theta;
     
     // a vector is created to hold predicted map locations within range of the particle
     vector<LandmarkObs> pred;
     
    //list of observations transformed from vehicle coordinates to maps coordinates 
    vector<LandmarkObs> transformed_observations;
    vector<LandmarkObs> predictions;
     
     // for every map landmark
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) 
    {
      // get id and x,y coordinates
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;
      

      // create a normal (Gaussian) distribution for x, y and theta
      //normal_distribution<double> dist_x(x_part, std_x);
      //normal_distribution<double> dist_y(y_part, std_y);
      //normal_distribution<double> dist_theta(theta_part, std_theta);
      
      // ensuring each map landmark is inside snesor range      
      if (fabs(landmark_x - x_part) <= sensor_range && fabs(landmark_y - y_part) <= sensor_range) 
      {
         // add prediction to vector
        predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y }); 
      
      }  
      
    }    
     
     for (unsigned int j = 0; j < transformed_observations.size(); j++) 
     {
      
      // 
      double obs_x, obs_y, pred_x, pred_y;
      obs_x = transformed_observations[j].x;
      obs_y = transformed_observations[j].y;

      int associated_prediction = transformed_observations[j].id;

      // get x,y coordinates prediction associated with the current observation
      for (unsigned int k = 0; k < predictions.size(); k++) 
      {
        if (predictions[k].id == associated_prediction) {
          pred_x = predictions[k].x;
          pred_y = predictions[k].y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      // calculate normalization term
      double gauss_norm, mu_x, mu_y;
      mu_x = 2;
      mu_y = 1;
      gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
      // calculate exponent
      double exponent;
      exponent = (pow(obs_x - mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(obs_y - mu_y, 2) / (2 * pow(sig_y, 2)));
      // product of this obersvation weight with total observations weight
      double weight;
      weight = gauss_norm * exp(-exponent);
      particles[i].weight *= weight;
       
     
     
   }
}
    

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  
  
  
  
  //vector<Particle> new_particles;

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

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
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
