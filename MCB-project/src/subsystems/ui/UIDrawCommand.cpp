#include "UIDrawCommand.hpp"

//this cpp file doesn't have much, might move stuff in here to the hpp and not have a cpp
namespace commands {

UIDrawCommand::UIDrawCommand(UISubsystem* subsystem) : subsystem(subsystem) {
    addSubsystemRequirement(subsystem);
    addGraphicsObject(&testGraphics);
    // addGraphicsObject(&testFill);
    addGraphicsObject(&laneAssistLines);
}

void UIDrawCommand::initialize() { 
    resetIteration();
    
    subsystem->setTopLevelContainer(this); 
}

void UIDrawCommand::execute() {}

void UIDrawCommand::end(bool) { subsystem->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands