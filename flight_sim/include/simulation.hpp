// Orchestrator for simluation
// Put stuff in here and test stuff in here accordingly

#pragma once

#include <iostream>
#include "physics_body.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Simulation {

    private:

        // Fields
        double delta_t;
        double t;
        
        // Example Environmental Constants
        // And other forces...
        Eigen::Vector3d wind;
        Eigen::Vector3d gravity;

    public:
        // Constructors
        Simulation() : delta_t(0.01), t(0.0), wind(Eigen::Vector3d::Zero()), gravity(Eigen::Vector3d(0, 0, -9.81)) {}

        double getDeltaTime() const { return delta_t; }
        double getTime() const { return t; }
        Eigen::Vector3d getWind() const { return wind; }
        Eigen::Vector3d getGravity() const { return gravity; }

        void setDeltaTime(double dt) { delta_t = dt; }
        void setWind(const Eigen::Vector3d& windVelocity) { wind = windVelocity; }
        void setGravity(const Eigen::Vector3d& gravityVector) { gravity = gravityVector; }
        void advance() { t += delta_t; }
        Eigen::Vector3d gravityForceFor(const PhysicsBody& body) const { return gravity * body.getMass(); }
        void applyGravity(PhysicsBody& body) const { body.applyForce(gravityForceFor(body)); }

};
