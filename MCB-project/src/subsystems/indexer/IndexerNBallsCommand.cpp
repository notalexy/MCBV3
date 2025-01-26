#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    indexer->resetBallsCounter();
    indexer->indexAtRate(ballsPerSecond);
}
void IndexerNBallsCommand::execute()
{
}

void IndexerNBallsCommand::end(bool) {
    indexer->resetBallsCounter();
    indexer->indexAtRate(0);
}

bool IndexerNBallsCommand::isFinished(void) const {
    if (numBalls < 0) {
        return false;
    }

    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands