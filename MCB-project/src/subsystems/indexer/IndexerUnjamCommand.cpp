#include "IndexerUnjamCommand.hpp"

namespace commands
{

void IndexerUnjamCommand::initialize() {
}

void IndexerUnjamCommand::execute()
{
    indexer->indexAtRate(IndexerSubsystem::UNJAM_BALL_PER_SECOND);
}

void IndexerUnjamCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerUnjamCommand::isFinished(void) const { return false; }
}  // namespace commands