package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Subsystems.EndEffector;
import frc.robot.commands.TeleopAutomation.DriveToPose;
import frc.robot.Subsystems.Elevator.ElevatorState;

public class AdaptableCommand extends Command {

    private CommandXboxController controller;
    private Supplier<ReefPoleScoringPoses> pole;
    private Supplier<ElevatorState> level;
    private Supplier<Boolean> source;
    private Pose2d goalPose;
    private boolean intaking;

    
    public AdaptableCommand(CommandXboxController controller, Supplier<ReefPoleScoringPoses> pole, Supplier<ElevatorState> level) {
        this.controller = controller;
        this.pole = pole;
        this.level = level;
        intaking = false;
    }

    public AdaptableCommand(Supplier<Boolean> intaking){
      this.intaking = true;
      this.source = source;

    }

    public void end() {
      
      if(intaking){
        if(!source.get()){
          goalPose = new Pose2d(1.191, 7.0298, Rotation2d.fromRadians(-0.939839628289));
        } else{
          goalPose = new Pose2d(1.1702, 0.9921, Rotation2d.fromRadians(0.9404398081));
        }
        if(Constants.alliance == Alliance.Red){
          goalPose.rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180));
        }

        new DriveToPose(() -> goalPose).schedule();

      } else{
        CommandFactory.AutoPoleAlign(level, pole, controller).schedule();
      }

      
      

    }
  
    public boolean isFinished() {
      return Math.abs(controller.getLeftX()) > 0.1 || Math.abs(controller.getLeftY()) > 0.1;
    }
}