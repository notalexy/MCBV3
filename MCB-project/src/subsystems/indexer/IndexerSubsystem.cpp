#include "IndexerSubsystem.hpp"

namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers)
      {}

void IndexerSubsystem::initialize() {
  motor_Indexer.initialize();
}

void IndexerSubsystem::refresh() {
    
}
} //namespace subsystems