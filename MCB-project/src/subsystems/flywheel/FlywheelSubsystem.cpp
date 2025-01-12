#include "FlywheelSubsystem.h"

namespace subsystems {
    FlyWheelSubsystem::FlyWheelSubsystem(tap::Drivers* drivers) 
      : tap::control::Subsystem(drivers),
        drivers(drivers) {} 

    void ShooterController::initialize() {
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
        // indexPIDController.runControllerDerivateError(indexerVoltage - motor_Indexer.getShaftRPM(), 1);
        // motor_Indexer.setDesiredOutput(static_cast<int32_t>(indexPIDController.getOutput()));

        flywheelPIDController1.runControllerDerivateError(flyWheelVoltage - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(static_cast<int32_t>(flywheelPIDController1.getOutput()));

        flywheelPIDController2.runControllerDerivateError(flyWheelVoltage - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(static_cast<int32_t>(flywheelPIDController2.getOutput()));

        if(servoTimer.execute()) hopperServo.updateSendPwmRamp();
    }

    void FlyWheelSubsystem::stopMotors() {
        //indexPIDController.runControllerDerivateError(0 - motor_Indexer.getShaftRPM(), 1);
        // motor_Indexer.setDesiredOutput(0);//static_cast<int32_t>(indexPIDController.getOutput()));

       // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController1.getOutput()));

       // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController2.getOutput()));

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        //TODO: Add the other motors
    }

    void FlyWheelSubsystem::enableShooting() {
        this->shootingSafety = true;
    }

    void FlyWheelSubsystem::disableShooting() {
        this->shootingSafety = false;
    }

    int FlyWheelSubsystem::getFlywheelVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return FLYWHEEL_MOTOR_MAX_SPEED;
        }else{
            return 0;
        }
    }

    // int FlyWheelSubsystem::getIndexerVoltage() {
    //     if (robotDisabled) return 0;
    //     if(shootingSafety){
    //         return indexerVoltage;
    //     }else{
    //         return 0;
    //     }
    // }

    // void FlyWheelSubsystem::setIndexer(double val) {
    //     indexerVoltage = val*INDEXER_MOTOR_MAX_SPEED;
    // }

    void FlyWheelSubsystem::disable(){
        robotDisabled = true;
    }
    void FlyWheelSubsystem::enable(){
        robotDisabled = false;
    }
    
    void FlyWheelSubsystem::shoot(double maxFrequency) {
        enableShooting();
        closeServo();  // when shooting resumes close servo to prevent balls from leaving
        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;

        double latency = 0.4, remaining = turretData.heatLimit - turretData.heat17ID1;
        // Check if the firing rate should be limited
        if (drivers->refSerial.getRefSerialReceivingData() && (10.0 * maxFrequency - turretData.coolingRate) * latency > remaining) {
            // Set the firing speed to C/10 Hz
            maxFrequency = turretData.coolingRate / 10.0;
        }

        setIndexer(maxFrequency / 20.0);
    }

    void FlyWheelSubsystem::setServo(float val) { if(secondTimer.execute()) hopperServo.setTargetPwm(val); }
}  // namespace subsystems