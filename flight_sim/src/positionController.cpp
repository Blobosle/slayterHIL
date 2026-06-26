#include <flight_sim.hpp>

positionController::positionController() :
    kp(Eigen::Vector3d::Zero()),
    desiredPos(Eigen::Vector3d::Zero()),
    maxVelocity(0.0)
{}

positionController::positionController(const Eigen::Vector3d& p) :
    kp(p),
    desiredPos(Eigen::Vector3d::Zero()),
    maxVelocity(0.0)
{}

void positionController::setTarget(const Eigen::Vector3d& target) {
    desiredPos = target;
}

void positionController::setMaxVelocity(double velocityLimit) {
    maxVelocity = velocityLimit;
}

Eigen::Vector3d positionController::getTarget() {
    return desiredPos;
}

Eigen::Vector3d positionController::getPositionError(const Eigen::Vector3d& currentPos) const {
    return desiredPos - currentPos;
}

Eigen::Vector3d positionController::compute(const Eigen::Vector3d& currentPos, double dt) {
    Eigen::Vector3d pos_error = getPositionError(currentPos);
    Eigen::Vector3d targetVelocity = kp.cwiseProduct(pos_error);
    if (maxVelocity > 0.0 && targetVelocity.norm() > maxVelocity) {
        targetVelocity = targetVelocity.normalized() * maxVelocity;
    }
    return targetVelocity;
}
