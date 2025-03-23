#include "IndexerStopCommand.hpp"
#include "IndexerSubsystemConstants.hpp"


namespace commands
{

void IndexerStopCommand::initialize() {
}

void IndexerStopCommand::execute()
{
    indexer->stopIndex();
}

void IndexerStopCommand::end(bool) {
}

bool IndexerStopCommand::isFinished(void) const { return false; }

}  // namespace commands