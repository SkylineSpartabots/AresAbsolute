package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Subsystems.Elevator.ElevatorState;

public class PausedCommand extends Command {

    private CommandXboxController controller;
    private Command pausable; // This is the command that will be paused and resumed

    
    public PausedCommand(CommandXboxController controller, Command pausable) {
        this.controller = controller;
        this.pausable = pausable;
    }

    public void end() {
      pausable.schedule();
    }
  
    public boolean isFinished() {
      return controller.getLeftX() < 0.1 &&  Math.abs(controller.getLeftY()) < 0.1;
    }
}