/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Modified for submission by Ed Voas, Udacity Self-Driving Car Course
 */

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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 200;

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  default_random_engine gen;

  for (int i = 0; i < num_particles; ++i) {
    particles.push_back({
      .x = dist_x(gen),
      .y = dist_y(gen),
      .theta = dist_theta(gen),
      .weight = 1.0
    });
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  default_random_engine gen;

  for (auto &p: particles) {
    if (abs(yaw_rate) > 0.0001) {
      const double v1 = velocity / yaw_rate;
      const double y1 = yaw_rate * delta_t;

      p.x += v1 * (sin(p.theta + y1) - sin(p.theta));
      p.y += v1 * (cos(p.theta) - cos(p.theta + y1));
      p.theta += y1;
    } else {
      const double v1 = velocity * delta_t;
      p.x += v1 * cos(p.theta);
      p.y += v1 * sin(p.theta);
    }

    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);
  }
}

int ParticleFilter::find_closest_landmark(const LandmarkObs &observation, const std::vector<LandmarkObs> &landmarks) {
  double min_dist = numeric_limits<double>::max();
  int best_landmark_index = -1;

  for (int i = 0; i < landmarks.size(); ++i) {
    const LandmarkObs &l = landmarks[i];
    const double d = dist(observation.x, observation.y, l.x, l.y);
    if (d < min_dist) {
      min_dist = d;
      best_landmark_index = i;
    }
  }

  return best_landmark_index;
}

void ParticleFilter::dataAssociation(const std::vector<LandmarkObs> &predicted, std::vector<LandmarkObs> &observations) {
  for (auto &o: observations) {
    o.id = find_closest_landmark(o, predicted);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  const double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

  for (auto &p: particles) {

    // get all landmarks in range
    std::vector<LandmarkObs> nearby_landmarks;
    nearby_landmarks.reserve(map_landmarks.landmark_list.size());
    for (const auto &l: map_landmarks.landmark_list) {
      if (dist(l.x_f, l.y_f, p.x, p.y) <= sensor_range) {
        nearby_landmarks.push_back({
          .x = l.x_f,
          .y = l.y_f,
          .id = l.id_i
        });
      }
    }

    // transform all observations to map coords
    std::vector<LandmarkObs> transformed;
    transformed.reserve(observations.size());
    for (const auto &o: observations) {
      const auto cos_t = cos(p.theta);
      const auto sin_t = sin(p.theta);

      LandmarkObs temp = {
        .x = (cos_t * o.x) - (sin_t * o.y) + p.x,
        .y = (sin_t * o.x) + (cos_t * o.y) + p.y,
      };
      temp.id = find_closest_landmark(temp, nearby_landmarks);

      transformed.push_back(temp);
    }

    double weight = 1;

    for (auto obs: transformed) {
      LandmarkObs lm = nearby_landmarks[obs.id];

      const double exponent = ((obs.x - lm.x) * (obs.x - lm.x)) / (2 * std_landmark[0] * std_landmark[0]) + ((obs.y - lm.y) * (obs.y - lm.y)) / (2 * std_landmark[1] * std_landmark[1]);

      weight *= gauss_norm * exp(-exponent);
    }

    p.weight = weight;
  }
}

void ParticleFilter::resample() {
  std::vector<double> weights;
  for (auto p: particles) {
    weights.push_back(p.weight);
  }

  default_random_engine gen;
  discrete_distribution<int> dist(weights.begin(), weights.end());

  std::vector<Particle> resampled;
  resampled.reserve(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    resampled.push_back(particles[dist(gen)]);
  }

  particles = resampled;
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
