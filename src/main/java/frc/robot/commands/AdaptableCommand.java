package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AdaptableCommand extends Command {

    CommandXboxController controller;
    
    public AdaptableCommand(CommandXboxController controller) {
        this.controller = controller;
    }

    public void end() {
      // CommandFactory.AutoPathScoreCoralWithDefender(null, null).schedule();
    }
  
    public boolean isFinished() {
      return controller.b().getAsBoolean();
    }
}