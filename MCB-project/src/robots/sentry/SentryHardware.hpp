#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/indexer/DoubleIndexerSubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

#include "drivers.hpp"

using namespace tap::motor;
using namespace tap::can;
namespace robots
{
//class for sentry robot. This has hardware and subsystems that are robot specific
class SentryHardware
{
public:
    SentryHardware(src::Drivers* drivers) : drivers(drivers) {}

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR3, CanBus::CAN_BUS2, false, "TopFlywheel", 0, 0};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR2, CanBus::CAN_BUS2, false, "BottomFlywheel", 0, 0};

    DjiMotor yawMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "Yaw", 0, 0};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR5, CanBus::CAN_BUS2, true, "Pitch", 0, 0};

    DjiMotor indexMotor1{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, false, "LeftIndexer", 0, 0};
    DjiMotor indexMotor2{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, true, "RightIndexer", 0, 0};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, false, "Motor 4 as 1", 0, 0};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "Motor 1 as 2", 0, 0};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, false, "Motor 2 as 3", 0, 0};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, false, "Motor 3 as 4", 0, 0};

};

}  // namespace ThornBots
