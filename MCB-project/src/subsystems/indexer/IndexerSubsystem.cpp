#include "IndexerSubsystem.hpp"

namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers)
    //   motor_Yaw(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "Yaw", 0, 0),
    //   motor_Pitch(drivers, tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "Pitch", 0, 0) 
      {}

void IndexerSubsystem::refresh() {
    
}
} //namespace subsystems