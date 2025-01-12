#include "FlywheelSubsystem.h"

namespace subsystems {
    FlyWheelSubsystem::FlyWheelSubsystem(tap::Drivers* drivers) 
      : tap::control::Subsystem(drivers),
        drivers(drivers) {} 

    void FlyWheelSubsystem::initialize() {
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
    }
    void FlyWheelSubsystem::updateSpeeds(){
        if(shooterControllerTimer.execute()) {
            // indexerVoltage = getIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
        }
    }

    void FlyWheelSubsystem::setMotorSpeeds() {
        updateSpeeds();

        flywheelPIDController1.runControllerDerivateError(flyWheelVoltage - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(static_cast<int32_t>(flywheelPIDController1.getOutput()));

        flywheelPIDController2.runControllerDerivateError(flyWheelVoltage - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(static_cast<int32_t>(flywheelPIDController2.getOutput()));
    }

    void FlyWheelSubsystem::stopMotors() {
       // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController1.getOutput()));

       // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController2.getOutput()));

        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }

    int FlyWheelSubsystem::getFlywheelVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return FLYWHEEL_MOTOR_MAX_SPEED;
        }else{
            return 0;
        }
    }

    void FlyWheelSubsystem::disable(){
        robotDisabled = true;
    }
    void FlyWheelSubsystem::enable(){
        robotDisabled = false;
    }
}  // namespace subsystems