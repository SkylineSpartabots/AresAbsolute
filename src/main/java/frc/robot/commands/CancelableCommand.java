package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CancelableCommand extends Command {

    Timer timer = new Timer();

    CommandXboxController controller;
    
    public CancelableCommand(CommandXboxController controller) {
      timer.restart();
        this.controller = controller;
    }
  
    public boolean isFinished() {
      return timer.hasElapsed(0.15) && controller.getLeftY() > Constants.stickDeadband || controller.getLeftX() > Constants.stickDeadband || controller.getRightX() > Constants.stickDeadband;
    }
}