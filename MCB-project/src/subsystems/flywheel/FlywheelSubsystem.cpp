#include "FlywheelSubsystem.h"

namespace subsystems {
    FlyWheelSubsystem::FlyWheelSubsystem(tap::Drivers* drivers) 
      : tap::control::Subsystem(drivers),
        drivers(drivers) {} 

        /**
         * initializes the 2 flywheels
         */
    void FlyWheelSubsystem::initialize() {
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
    }
    
    /**
     * sets the target motor RPM of the flywheels
     */
    void FlyWheelSubsystem::setTargetMotorRPM(int targetMotorRPM){
        this->targetMotorRPM = targetMotorRPM;
        flywheelPIDController1.runControllerDerivateError(targetMotorRPM - motor_Flywheel1.getShaftRPM(), 1);
        flywheelPIDController2.runControllerDerivateError(targetMotorRPM - motor_Flywheel2.getShaftRPM(), 1);

        flyWheel1Voltage = static_cast<int32_t>(flywheelPIDController1.getOutput());
        flyWheel2Voltage = static_cast<int32_t>(flywheelPIDController2.getOutput());
    }

    /**
     * refreshes flywheel output
     */
    void FlyWheelSubsystem::refresh() {
        motor_Flywheel1.setDesiredOutput(flyWheel1Voltage);
        motor_Flywheel2.setDesiredOutput(flyWheel2Voltage);
    }

    /**
     * turn off
     */
    void FlyWheelSubsystem::stopMotors() {
        flyWheel1Voltage = 0;
        flyWheel2Voltage = 0;
    }

    void FlyWheelSubsystem::disable(){
        robotDisabled = true;
    }
    void FlyWheelSubsystem::enable(){
        robotDisabled = false;
    }
}  // namespace subsystems