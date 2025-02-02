#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/indexer/IndexerSubsystem.h"

#include "drivers.h"

namespace commands
{
using subsystems::IndexerSubsystem;
using tap::communication::serial::Remote;

class IndexerUnjamCommand : public tap::control::Command
{
public:
    IndexerUnjamCommand(src::Drivers* drivers, IndexerSubsystem* indexer)
        : drivers(drivers),
          indexer(indexer)
    {
        addSubsystemRequirement(indexer);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "indexer unjam command"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
};
}  // namespace commands