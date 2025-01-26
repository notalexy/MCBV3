#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    indexer->resetBallsCounter();
}
void IndexerNBallsCommand::execute()
{
        indexer->indexAtRate(ballsPerSecond);
}

void IndexerNBallsCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerNBallsCommand::isFinished(void) const {
    if (numBalls < 0) {
        return false;
    }

    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands