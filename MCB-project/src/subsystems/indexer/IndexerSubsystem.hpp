#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "drivers.hpp"

namespace subsystems
{

class IndexerSubsystem : public tap::control::Subsystem
{
public:  // Public Variables
constexpr static int INDEXER_MOTOR_MAX_SPEED = 6177; // With the 2006, this should give
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig pid_conf_index = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};

protected:  // Private Variables
tap::Drivers* drivers;
// #if defined(sentry)
tap::motor::DjiMotor motor_Indexer;//{drivers, tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0};
// #else
// tap::motor::DjiMotor motor_Indexer{drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0};
// #endif
tap::algorithms::SmoothPid indexPIDController{pid_conf_index};

float ballsPerSecond = 0.0f;
int32_t indexerVoltage = 0;
int64_t numTicksAtInit = 0;

public:  // Public Methods

IndexerSubsystem(tap::Drivers* drivers);

~IndexerSubsystem() {}

void initialize();

void refresh() override;

void indexAtRate(float ballsPerSecond);

void setTargetMotorRPM(int targetMotorRPM);

void stopIndex();

float getNumBallsShot();

void resetBallsCounter();

float getBallsPerSecond();

private:  // Private Methods
};
} //namespace subsystems