#include "UIDrawCommand.hpp"

// this cpp file doesn't have much, might move stuff in here to the hpp and not have a cpp
namespace commands {

<<<<<<< HEAD
UIDrawCommand::UIDrawCommand(UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain)
    : ui(ui),
      gimbal(gimbal),
      flywheel(flywheel),
      indexer(indexer),
      drivetrain(drivetrain),
      laneAssistLines(gimbal) {
    addSubsystemRequirement(ui);
    // addGraphicsObject(&testGraphics);
=======
UIDrawCommand::UIDrawCommand(UISubsystem* subsystem) : subsystem(subsystem) {
    addSubsystemRequirement(subsystem);
    addGraphicsObject(&testGraphics);
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9
    // addGraphicsObject(&testFill);
    addGraphicsObject(&laneAssistLines);
}

<<<<<<< HEAD
void UIDrawCommand::initialize() { ui->setTopLevelContainer(this); }
=======
void UIDrawCommand::initialize() { 
    resetIteration();
    
    subsystem->setTopLevelContainer(this); 
}
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9

void UIDrawCommand::execute() {}

void UIDrawCommand::end(bool) { ui->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands