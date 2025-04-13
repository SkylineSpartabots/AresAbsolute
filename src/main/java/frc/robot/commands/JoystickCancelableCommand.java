package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class JoystickCancelableCommand extends Command {

    CommandXboxController controller;
    
    public JoystickCancelableCommand(CommandXboxController controller) {
        this.controller = controller;
    }
  
    public boolean isFinished() {
      return Math.abs(controller.getLeftX()) > 0.1;
    }
}