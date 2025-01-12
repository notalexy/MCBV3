#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "drivers.hpp"

namespace subsystems {
    static tap::arch::PeriodicMilliTimer shooterControllerTimer(2);
    static tap::arch::PeriodicMilliTimer secondTimer(100);
    static tap::arch::PeriodicMilliTimer servoTimer(20);

    class IndexerSubsystem : public tap::control::Subsystem {
        public:  // Public Variables
            // constexpr static double PI = 3.14159;
            
            constexpr static int INDEXER_MOTOR_MAX_SPEED = 6177;   // With the 2006, this should give
            constexpr static tap::algorithms::SmoothPidConfig pid_conf_flywheel = {40, 0.1, 0, 10.0, 10000, 1, 0, 1, 0, 0, 0};
           constexpr static tap::algorithms::SmoothPidConfig pid_conf_index = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};

        private:  // Private Variables
            tap::Drivers* drivers;
            // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
            tap::motor::DjiMotor motor_Indexer =
                tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0);

            tap::algorithms::SmoothPid indexPIDController = tap::algorithms::SmoothPid(pid_conf_index);

            double flyWheelVoltage, indexerVoltage = 0.0;

        public:  // Public Methods

        IndexerSubsystem(tap::Drivers* drivers);

        void refresh() override;
        
        void setIndexer(double val);
        
        inline void idle() { setIndexer(0); }
        inline void unjam() { setIndexer(-0.1); }

        private:  // Private Methods
            int getFlywheelVoltage();
            int getIndexerVoltage();
            int getServoPosition();
    };
} //namespace subsystems