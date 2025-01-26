#include "IndexerUnjamCommand.hpp"

namespace commands
{

void IndexerUnjamCommand::initialize() {
    indexer->indexAtRate(IndexerSubsystem::UNJAM_BALL_PER_SECOND);
}
void IndexerUnjamCommand::execute()
{
}

void IndexerUnjamCommand::end(bool) {
    indexer->indexAtRate(0);
}

bool IndexerUnjamCommand::isFinished(void) const { return false; }
}  // namespace commands