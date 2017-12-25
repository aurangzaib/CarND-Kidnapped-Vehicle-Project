/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle_filter.h"

using namespace std;

int indexOfSmallestElement(const vector<double> list) {
  int index = 0;
  for (int loop = 1; loop < list.size(); loop += 1) {
    if (list[loop] < list[index])
      index = loop;
  }
  return index;
}

void ParticleFilter::init(const double x, const double y, const double theta, const double std[]) {
  // Set the number of particles. Initialize all 
  //  particles to first position (based on estimates of
  //  x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.

  /*
   * x, y are GPS positions
   * theta is heading/orientation
   * std is uncertainties for x, y and theta
   */

  // set number of particles. Resize particles and weights 
  num_particles = 80;
  weights.resize(num_particles);
  particles.resize(num_particles);

  // Define random generator with Gaussian distribution
  random_device rd;
  std::default_random_engine g(rd());
  std::normal_distribution<double>  dist_x(x, std[0]),
                                    dist_y(y, std[1]),
                                    dist_theta(theta, std[2]);

  // set each particle position and orientation
  for (int loop = 0; loop < num_particles; loop += 1) {
    particles[loop].id = loop;
    particles[loop].x = dist_x(g);
    particles[loop].y = dist_y(g);
    particles[loop].theta = dist_theta(g);
    particles[loop].weight = 1.0;
  }

  // set initializatin flag
  is_initialized = true;
}

void ParticleFilter::prediction(const double delta_t, const double std[], const double velocity, const double yaw_rate) {
  // Add measurements to each particle
  // Add random Gaussian noise.

  // Define random generator with Gaussian distribution
  std::default_random_engine g;

  // generate gaussian distribution for sensor noise (uncertainties)
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  for (auto &particle: particles) {

    // update position for each particle
    if (fabs(yaw_rate) != 0.0) {
      particle.x     += (velocity / yaw_rate) * (+sin(particle.theta + (yaw_rate * delta_t)) - sin(particle.theta));
      particle.y     += (velocity / yaw_rate) * (-cos(particle.theta + (yaw_rate * delta_t)) + cos(particle.theta));
      particle.theta += yaw_rate * delta_t;
    } else {
      particle.x     += velocity * cos(particle.theta) * delta_t;
      particle.y     += velocity * sin(particle.theta) * delta_t;
    }

    // add sensor noise to position for each particle
    particle.x       += dist_x(g);
    particle.y       += dist_y(g);
    particle.theta   += dist_theta(g);
  }
}

void ParticleFilter::updateWeights(const double sensor_range, 
                                   const double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

  // denominator term
  const double gauss_x_den = 2 * pow(std_landmark[0], 2);
  const double gauss_y_den = 2 * pow(std_landmark[1], 2);
  const double gauss_den   = 2 * M_PI * std_landmark[0] * std_landmark[1];

  // iterate particles
  for (auto &particle:particles) {
    double updated_weight = 1.0;

    // iterate observations for each particle
    for (auto const &observation: observations) {

      // transform observation from vehicle coordinate system (VCS) to map coordinate system (MCS)
      double observation_mcs_x = observation.x * cos(particle.theta) - observation.y * sin(particle.theta) + particle.x;
      double observation_mcs_y = observation.x * sin(particle.theta) + observation.y * cos(particle.theta) + particle.y;

      // find distances of an MCS observation to all map landmarks
      vector<double> distances;
      for (auto const &l: map_landmarks.landmark_list) {
        const double distance_from_particle = pow(particle.x - l.x_f, 2) + pow(particle.y - l.y_f, 2);
        double distance = 0;
        // penalize observations which are out of sensor range
        if (sqrt(distance_from_particle) <= sensor_range) {
          distance = pow(observation_mcs_x - l.x_f, 2) + pow(observation_mcs_y - l.y_f, 2);
        } else {
          distance = 9999999.999;
        }
        distances.push_back(sqrt(distance));
      }

      // distance and associated landmark for MCS observation with minimum distance
      const auto index = indexOfSmallestElement(distances);
      const auto associated_landmark = map_landmarks.landmark_list[index];

      // argument of exponential term
      double exp_arg = 0.0;
      exp_arg += pow(observation_mcs_x - associated_landmark.x_f, 2) / gauss_x_den;
      exp_arg += pow(observation_mcs_y - associated_landmark.y_f, 2) / gauss_y_den;

      // update weights with normzalization of all observations
      updated_weight *= exp(-exp_arg) / gauss_den;
    }

    // combine weights of all observations for given particle
    particle.weight = updated_weight;

    // index of particle (range-based for)
    const auto index = &particle - &particles[0];
    
    // update weight
    weights[index] = particle.weight;
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight.

  // initialise engine
  std::random_device rd;
  default_random_engine g(rd());
  vector<Particle> resampled_particles(num_particles);
  for (auto &particle: resampled_particles) {
    discrete_distribution<int> sample_index(weights.begin(), weights.end());
    particle = particles[sample_index(g)];
  }

  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x,
                                         std::vector<double> sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
