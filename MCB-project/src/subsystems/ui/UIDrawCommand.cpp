#include "UIDrawCommand.hpp"

namespace commands {


void UIDrawCommand::initialize() {
    addGraphicsObject(&laneAssistLines);
    // add more here
}

void UIDrawCommand::execute() { subsystem->setTopLevelContainer(this); }

void UIDrawCommand::end(bool) { subsystem->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands