#include "UIDrawCommand.hpp"

namespace commands {

void UIDrawCommand::initialize() {
    // we can't use new here because we don't want to reconstruct the top level graphics objects if the command is stopped and restarted
    addGraphicsObject(&laneAssistLines);
}

void UIDrawCommand::execute() { subsystem->setTopLevelContainer(this); }

void UIDrawCommand::end(bool) { subsystem->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands