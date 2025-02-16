#include "FlywheelSubsystem.hpp"

namespace subsystems {
    FlywheelSubsystem::FlywheelSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* flywheel1, tap::motor::DjiMotor* flywheel2) 
      : tap::control::Subsystem(drivers),
        motor_Flywheel1(flywheel1),
        motor_Flywheel2(flywheel2),
        drivers(drivers) {} 

        /**
         * initializes the 2 flywheels
         */
    void FlywheelSubsystem::initialize() {
        motor_Flywheel1->initialize();
        motor_Flywheel2->initialize();
    }
    
    /**
     * sets the target motor RPM of the flywheels
     */
    void FlywheelSubsystem::setTargetVelocity(int targetMotorRPM){
        this->targetMotorRPM = targetMotorRPM;
        flywheelPIDController1.runControllerDerivateError(targetMotorRPM - motor_Flywheel1->getShaftRPM(), 1);
        flywheelPIDController2.runControllerDerivateError(targetMotorRPM - motor_Flywheel2->getShaftRPM(), 1);

        flyWheel1Voltage = static_cast<int32_t>(flywheelPIDController1.getOutput());
        flyWheel2Voltage = static_cast<int32_t>(flywheelPIDController2.getOutput());
    }

    /**
     * refreshes flywheel output
     */
    void FlywheelSubsystem::refresh() {
        motor_Flywheel1->setDesiredOutput(flyWheel1Voltage);
        motor_Flywheel2->setDesiredOutput(flyWheel2Voltage);
    }

    /**
     * turn off
     */
    void FlywheelSubsystem::stopMotors() {
        flyWheel1Voltage = 0;
        flyWheel2Voltage = 0;
    }

    float FlywheelSubsystem::getShootingVelocity() {
        return targetMotorRPM * 2 * (FLYWHEEL_RADIUS_MM / 1000.0f) * PI / 60.0f;
    }

}  // namespace subsystems