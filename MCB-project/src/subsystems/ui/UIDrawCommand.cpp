#include "UIDrawCommand.hpp"

namespace commands {

void UIDrawCommand::initialize() { 
    subsystem->setTopLevelContainer(this); 

    addGraphicsObject(&laneAssistLines);
}

void UIDrawCommand::execute() {}

void UIDrawCommand::end(bool) { subsystem->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands