/**
 * This is a library to convert from force, moment setpoints to individual motor commands
 *
 *
 */

#pragma once

#include <matrix/matrix/math.hpp>

/**
 * dx is the abs, perpendicular distance along the x axis to the y axis
 */
struct MotorPos { float dx; float dy; };
struct GeometryMotors {
    MotorPos motor1;
    MotorPos motor2;
    MotorPos motor3;
    MotorPos motor4;
};

class ArplControlAllocator
{
public:
    ArplControlAllocator();

    void generateMixerMatrix(); // to be called every time the parameters are updated
    void updateActuatorSetpoint(float &thrust_sp, matrix::Vector3<float> &moment_sp);
    const matrix::Vector<float, 4> getActuatorSp();
    float getThrustNewtons(float thrust);
    float normalizeActuatorSetpoint(float actuator_sp);
    float getRpmfromThrust(float &rpm);
    void setKfandKm(float &kf_param, float &km_param) {   _kf = kf_param; _km = km_param;    }
    void setGradientandOffset(float &grad_param, float &offset_param)  {  _gradient = grad_param; _offset = offset_param; }
    void setMinMaxRpm(float &min_rpm,float &max_rpm) {_min_rpm = min_rpm; _max_rpm = max_rpm;   };
    void setMotorMomentArmParams(const GeometryMotors &motors);


private:

    // everything for parameters
    float _kf{0.0};
    float _km{0.0};
    matrix::Matrix<float, 4, 2> _rotors_pos; // for each rotor this stores the length of the x y moment arms
    float _gradient{0.0};
    float _offset{0.0};
    float _max_rpm{0.0};
    float _min_rpm{0.0};

    // calculated values
    float _thrust{0.0}; // thrust in newtons
    matrix::SquareMatrix<float, 4> _mixer_matrix;
    matrix::SquareMatrix<float, 4> _mixer_mat_inv;
    matrix::Vector<float, 4> _actuator_sp; // normalized inputs
    GeometryMotors _motors;

};