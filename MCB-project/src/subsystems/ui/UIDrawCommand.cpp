#include "UIDrawCommand.hpp"

namespace commands {

void UIDrawCommand::initialize() { 
    subsystem->setTopLevelContainer(this); 

    addGraphicsObject(&laneAssistLines);
    //add more here
}

void UIDrawCommand::execute() {}

void UIDrawCommand::end(bool) { subsystem->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands