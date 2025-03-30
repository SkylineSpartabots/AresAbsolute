// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomation;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IggingIt extends Command {
  private final CommandSwerveDrivetrain s_Swerve;
  private ReefSidePositions targetReefSide;
  PathPlannerPath path;
  public IggingIt(ReefSidePositions targetReefSide) {
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    addRequirements(s_Swerve);
    this.targetReefSide = targetReefSide;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        s_Swerve.getPose(),
        targetReefSide.getPose());
     
    path = new PathPlannerPath(
      waypoints,
      new PathConstraints(
                Constants.MaxSpeed + 1,
                Constants.MaxAcceleration + 4,
                Constants.MaxAngularVelocity,
                Constants.MaxAngularRate
        ),
      null,
      new GoalEndState(0, targetReefSide.getPose().getRotation())
    );
    path.preventFlipping = true;
    new Trajectory().f
        

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
