package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class CancelableCommand extends Command {

    CommandXboxController controller;
    
    public CancelableCommand(CommandXboxController controller) {
        this.controller = controller;
    }
  
    public boolean isFinished() {
      return controller.getLeftY() > 0.1 || controller.getLeftX() > 0.1 || controller.getRightX() > 0.1;
    }
}