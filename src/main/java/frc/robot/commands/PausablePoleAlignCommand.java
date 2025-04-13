package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Subsystems.Elevator.ElevatorState;

public class PausablePoleAlignCommand extends Command {

    private CommandXboxController controller;
    
    public PausablePoleAlignCommand(CommandXboxController controller) {
        this.controller = controller;
    }

    public void end() {
      new PausedCommand(controller, CommandFactory.AutoPoleAlign(controller)).schedule();
    }
  
    public boolean isFinished() {
      return Math.abs(controller.getLeftX()) > 0.1 || Math.abs(controller.getLeftY()) > 0.1;
    }
}