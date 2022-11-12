#include "ArplControlAllocator.hpp"

/**
 * Regenerate the mixer matrix and it's inverse if the relevant parameters are updated.
 * Keep in mind that px4 uses NED which effects the moment equations about y and z axes
 */
void ArplControlAllocator::generateMixerMatrix() {

    matrix::SquareMatrix<float, 4> mixer_mat;
    float km_kf = _km / _kf;
    // fill in mixer mat
    /**
     * [1, 1, 1, 1]
     * [] Mx
     * [] My
     * [] Mz
     */

    // params needed: each motor dx and dy, kf, km
    matrix::Vector<float, 4> vec;
    vec.setAll(1.0f);
    mixer_mat.row(0) = vec;

    // mx = m1 + m2 - m0 - m3
    float moment_x[4] = {-_motors.motor1.dy,
                         _motors.motor2.dy,
                         _motors.motor3.dy,
                         -_motors.motor4.dy  };

    // my = m0 + m2 - m1 - m3
    float moment_y[4] = {_motors.motor1.dx,
                         -_motors.motor2.dx,
                         _motors.motor3.dx,
                         -_motors.motor4.dx  };

    //
    float moment_z[4] = {km_kf,
                         km_kf,
                         -km_kf,
                         -km_kf};

    mixer_mat.row(1) = matrix::Vector<float, 4> (moment_x);
    mixer_mat.row(2) = matrix::Vector<float, 4> (moment_y);
    mixer_mat.row(3) = matrix::Vector<float, 4> (moment_z);

    _mixer_matrix = mixer_mat;
    _mixer_mat_inv = inv(mixer_mat);
}

/**
 *
 * @param thrust normalized thrust [0, 1]
 * @return thrust in newtons
 */
float ArplControlAllocator::getThrustNewtons(float thrust) {

    // applying the inverse mapping to get thrust in newtons from commanded throttle
    // normal mapping is ans = gradient * x + offset
    // we need x = (ans - offset) / gradient ; where x is the rpm
    // thrust(N) = kf * rpm ** 2;

    return _kf * powf(((thrust - _offset) / _gradient), 2.0f);
}

/**
 *
 * @return normalized actuator setpoints
 */
const matrix::Vector<float, 4> ArplControlAllocator::getActuatorSp() {
    return _actuator_sp;
}

/**
 *
 * @param thrust_sp normalized thrust sp
 * @param moment_sp
 */
void ArplControlAllocator::updateActuatorSetpoint(float &thrust_sp, matrix::Vector3<float> &moment_sp) {

    float thrust = getThrustNewtons(thrust_sp);
    matrix::Vector<float, 4> force_moments;
    force_moments(0) = thrust;
    force_moments(1) = moment_sp(0);
    force_moments(2) = moment_sp(1);
    force_moments(3) = moment_sp(2);
    matrix::Vector<float, 4> actuator_sp = _mixer_mat_inv * force_moments;
    matrix::Vector<float, 4> actuator_sp_rpm;

    for(int i = 0; i < 4; i++){
        actuator_sp_rpm(i) = getRpmfromThrust(actuator_sp(i));
        actuator_sp(i) = normalizeActuatorSetpoint(actuator_sp_rpm(i));
    }

    _actuator_sp = actuator_sp;

}

/**
 *
 * @param actuator_sp this is the motor angular rate in rpm
 * @return normalized set point [0, 1]
 */
float ArplControlAllocator::normalizeActuatorSetpoint(float actuator_sp) {

    return actuator_sp * _gradient + _offset;
}

float ArplControlAllocator::getRpmfromThrust(float &thrust) {

    return powf(fabsf(thrust / _kf), 0.5f);

}

void ArplControlAllocator::setMotorMomentArmParams(const GeometryMotors &motors) {

    _motors = motors;
    generateMixerMatrix();
}
