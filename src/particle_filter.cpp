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

In the code above, you seem to be at step 2 but there has to be a condition before you push data to predicted vector about whether those map landmarks are inside sensor range.

Particle weight would be normalized according to gaussian normalization later towards step 4.
   
   
   */
// To keep track of each particle
  for(int i = 0; i < num_particles; i++)
    {	
     	double x_part = particles[i].x;
 		double y_part  = particles[i].y;
 		double theta_part = particles[i].theta;

// To keep track of each landmark
   		float landmark_x;          
    	float landmark_y;
    	int landmark_id;

// a vector created to hold predicted map locations within sensor range of the particle
 		vector<LandmarkObs> predictions;

//Each map landmark for loop
        for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) 
		{

// get id and x,y coordinates for each landmark
			landmark_x = map_landmarks.landmark_list[j].x_f;
			landmark_y = map_landmarks.landmark_list[j].y_f;
			landmark_id = map_landmarks.landmark_list[j].id_i;

//Only consider landmarks within sensor range of the particle (rather than using the "dist" method considering a circular region around the particle, this considers a rectangular region but is computationally faster)
			if(fabs(landmark_x - x_part) <= sensor_range && fabs(landmark_y - y_part) <= sensor_range)
            {
           		predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
			}
		}

// TRANSFORM each observation marker from the vehicle's coordinates to the map's coordinates
 		vector<LandmarkObs> transformed_observations;
 		for(unsigned int j = 0; j < observations.size(); j++) 
 		{
  			double t_x = cos(theta_part)*observations[j].x - sin(theta_part)*observations[j].y + x_part;
    		double t_y = sin(theta_part)*observations[j].x + cos(theta_part)*observations[j].y + y_part;
 			transformed_observations.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
 		}

// Nearest Neighbor Data Association applied 
 		dataAssociation(predictions, transformed_observations);

// reinitializing weight
 		particles[i].weight = 1.0;
// take in transformed observations one by one
 	for(unsigned int j = 0; j < transformed_observations.size(); j++) 
 	{
   		double o_x, o_y, pr_x, pr_y;
    	o_x = transformed_observations[j].x;
    	o_y = transformed_observations[j].y;
  		int associated_prediction = transformed_observations[j].id;

// take in x,y coordinates of the prediction associated with current observation
   		for(unsigned int k = 0; k < predictions.size(); k++) 
    	{
 		 	if(predictions[k].id == associated_prediction) 
  			{
 				pr_x = predictions[k].x;
  				pr_y = predictions[k].y;
  			}
 		}

// calculate weight for this observation with multivariate Gaussian
  		double sigma_x = std_landmark[0];
   		double sigma_y = std_landmark[1];

// calculate normalization term
      	double gauss_norm;
      	gauss_norm = 1 / (2 * M_PI * sigma_x * sigma_y);
     
// calculate exponent
      	double exponent;
      	exponent = (pow(pr_x - o_x, 2) / (2 * pow(sigma_x, 2))) + (pow(pr_y - o_y, 2) / (2 * pow(sigma_y, 2)));
     
 // product of this observation weight with total observations weight
      	double weight;
      	weight = gauss_norm * exp(-exponent);
      	particles[i].weight = particles[i].weight*weight;  
 		}
 	}
}
