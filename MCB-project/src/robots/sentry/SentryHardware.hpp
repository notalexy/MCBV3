#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"

#include "drivers.hpp"

using namespace tap::motor;
using namespace tap::can;
namespace robots
{
//class for standard robot. This has hardware and subsystems that are robot specific. This means we can have multiple StandardControl for different control schemes
class SentryHardware
{
public:
    SentryHardware(src::Drivers* drivers) : drivers(drivers) {}

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR3, CanBus::CAN_BUS2, true, "Flywheel", 0, 0};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR1, CanBus::CAN_BUS2, false, "Flywheel1", 0, 0};

    DjiMotor indexMotor1{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, false, "Indexer1", 0, 0};
    DjiMotor indexMotor2{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, true, "Indexer2", 0, 0};

    //subsystems
    subsystems::GimbalSubsystem gimbal{drivers};
    subsystems::FlywheelSubsystem flywheel{drivers, &flywheelMotor1, &flywheelMotor2};
    subsystems::IndexerSubsystem indexer{drivers};

};

}  // namespace ThornBots
