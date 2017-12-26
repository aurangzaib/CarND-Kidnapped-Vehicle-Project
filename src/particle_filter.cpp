/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <iostream>
#include <sstream>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(const double x, const double y, const double theta, const double std[]) {
  /*
   * x, y are GPS positions
   * theta is heading/orientation
   * std is uncertainties for x, y and theta
   *
   * Set the number of particles. Initialize all
   * particles to first position (based on estimates of
   * x, y, theta and their uncertainties from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.
   */

  /**************************************************************
   * STEP - 1:
   * Set number of particles. Resize particles and weights
   **************************************************************/

  num_particles = 60;
  weights.resize(num_particles);
  particles.resize(num_particles);

  /**************************************************************
   * STEP - 2:
   * Generate random noise
   **************************************************************/

  // Define random generator with Gaussian distribution
  random_device rd;
  std::default_random_engine g(rd());
  std::normal_distribution<double>  dist_x(x, std[0]),
                                    dist_y(y, std[1]),
                                    dist_theta(theta, std[2]);

  /**************************************************************
   * STEP - 3:
   * Set each particle position and orientation
   **************************************************************/

  for (auto &particle: particles) {
    particle.x = dist_x(g);
    particle.y = dist_y(g);
    particle.theta = dist_theta(g);
    particle.weight = 1.0;
  }

  // set initialization flag
  is_initialized = true;
}

void ParticleFilter::prediction(const double dt, const double std[], const double velocity, const double yaw_rate) {

  /**************************************************************
   * STEP - 1:
   * Generate adaptive white gaussian noise (AWGN)
   **************************************************************/

  // Define random generator with Gaussian distribution
  std::default_random_engine g;

  // generate gaussian distribution for sensor noise (uncertainties)
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  /**************************************************************
   * STEP - 2:
   * Update particle position and orientation
   * Motion Model: Constant Turn Rate and Velocity (CTRV)
   * Add noise
   **************************************************************/

  for (auto &particle: particles) {
    // with turn
    if (fabs(yaw_rate) > 0.00001) {
      const auto theta_pred = particle.theta + (yaw_rate * dt);
      particle.x     += (velocity / yaw_rate) * (+sin(theta_pred) - sin(particle.theta));
      particle.y     += (velocity / yaw_rate) * (-cos(theta_pred) + cos(particle.theta));
      particle.theta += yaw_rate * dt;
    // without turn
    } else {
      particle.x     += velocity * cos(particle.theta) * dt;
      particle.y     += velocity * sin(particle.theta) * dt;
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

  // denominator term
  const double gauss_x_den = 2 * pow(std_landmark[0], 2);
  const double gauss_y_den = 2 * pow(std_landmark[1], 2);
  const double gauss_den   = 2 * M_PI * std_landmark[0] * std_landmark[1];

  // iterate particles
  for (auto &particle:particles) {
    double updated_weight = 1.0;

    // iterate observations for each particle
    for (auto const &obs_vcs: observations) {

      // penalty when observation is out of range
      const double OUT_OF_RANGE_PENALTY = 9999999.999;

      /**************************************************************
       * STEP - 1:
       * Transformation observation
       * From vehicle coordinate system (VCS) to map coordinate system (MCS)
       **************************************************************/

      double obs_mcs_x = obs_vcs.x * cos(particle.theta) - obs_vcs.y * sin(particle.theta) + particle.x;
      double obs_mcs_y = obs_vcs.x * sin(particle.theta) + obs_vcs.y * cos(particle.theta) + particle.y;

      /**************************************************************
       * STEP - 2:
       * Nearest Landmark from Observation
       **************************************************************/

      // find distances of an MCS observation to all map landmarks
      vector<double> distances;
      for (auto const &l: map_landmarks.landmark_list) {
        const double dx = pow(particle.x - l.x_f, 2);
        const double dy = pow(particle.y - l.y_f, 2);
        const double distance_from_particle = sqrt(dx + dy);
        double distance = 0;

        // distance between lidar and landmark
        if (distance_from_particle <= sensor_range) {
          const double obs_dx = pow(obs_mcs_x - l.x_f, 2);
          const double obs_dy = pow(obs_mcs_y - l.y_f, 2);
          distance = sqrt(obs_dx + obs_dy);
        }

        // penalize observations which are out of sensor range
        else {
          distance = OUT_OF_RANGE_PENALTY;
        }
        // append distance
        distances.push_back(distance);
      }

      // associated landmark for MCS observation with minimum distance
      const auto index = indexOfSmallestElement(distances);
      const auto associated_landmark = map_landmarks.landmark_list[index];

      /**************************************************************
       * STEP - 3:
       * Find normalized probability using multi-variate Gaussian distribution
       **************************************************************/

      // argument of exponential term
      double exp_arg = 0.0;
      exp_arg += pow(obs_mcs_x - associated_landmark.x_f, 2) / gauss_x_den;
      exp_arg += pow(obs_mcs_y - associated_landmark.y_f, 2) / gauss_y_den;

      // update weights with normalization of all observations
      updated_weight *= exp(-exp_arg) / gauss_den;
    }

    /**************************************************************
     * STEP - 4:
     * Update Particle Weight
     **************************************************************/

    // update particle weight
    particle.weight = updated_weight;

    // index of particle (range-based for)
    const auto index = &particle - &particles[0];

    // update weight
    weights[index] = updated_weight;
  }
}

void ParticleFilter::resample() {

  /**************************************************************
   * STEP - 1:
   * Resample particles with replacement
   * Probability are proportional to their weight.
   **************************************************************/

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

Particle ParticleFilter::SetAssociations(Particle particle,
                                         const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  // Clear the previous associations
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
