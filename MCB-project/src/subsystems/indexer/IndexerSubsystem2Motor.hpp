#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

class IndexerWithSecondMotorSubsystem : public IndexerSubsystem
{

public:
    // Additional Motor Constants (if necessary)
    tap::motor::DjiMotor* motorIndexer2;

    IndexerWithSecondMotorSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2);

    ~IndexerWithSecondMotorSubsystem() {}

    void initialize() override;
    void refresh() override;
    void stopIndex() override;
    void indexAtRate(float ballsPerSecond) override;
    void setTargetMotorRPM(int targetMotorRPM) override;

private:
tap::algorithms::SmoothPid indexPIDController2{PID_CONF_INDEX};
int32_t indexerVoltage2 = 0;

};

} // namespace subsystems
