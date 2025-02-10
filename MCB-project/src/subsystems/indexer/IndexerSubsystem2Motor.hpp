#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

class IndexerWithSecondMotorSubsystem : public IndexerSubsystem
{
public:
    // Additional Motor Constants (if necessary)
    tap::motor::DjiMotor motor_Indexer2;

    IndexerWithSecondMotorSubsystem(tap::Drivers* drivers);

    ~IndexerWithSecondMotorSubsystem() {}

    void initialize() override;
    void refresh() override;
    void stopIndex();
};

} // namespace subsystems
