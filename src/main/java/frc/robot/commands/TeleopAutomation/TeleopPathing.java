// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomation;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopPathing extends Command {
  private Trajectory trajectory;
  private final CommandSwerveDrivetrain s_Swerve;
  private Optional<DriverStation.Alliance> alliance;
  private Optional<Pose2d> startPose;
  private Timer timer;
  private DriveControlSystems controlSystems;
  private RobotState robotState;
  private PIDController xController = new PIDController(3.6, 0, 0.02);
  private PIDController yController = new PIDController(3.6, 0, 0.02);
  private PIDController thetaController = new PIDController(1.4, 0, 0.02);

  public TeleopPathing(String name) {
    if (Choreo.loadTrajectory(name).isPresent()) {
      trajectory = Choreo.loadTrajectory(name).get();
    } else{
      System.out.println("AUTO BROKEN");
    }
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    alliance = DriverStation.getAlliance();
    timer = new Timer();
    controlSystems = DriveControlSystems.getInstance();
    robotState = RobotState.getInstance();
    addRequirements(s_Swerve);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    // if (trajectory != null){
    //   startPose = trajectory.getInitialPose(alliance.get() == DriverStation.Alliance.Red);
    //   s_Swerve.resetOdo(startPose.get());
    // }
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(trajectory != null){
      Optional<SwerveSample> sample = trajectory.sampleAt(timer.get(), alliance.get() == DriverStation.Alliance.Red);
      followAutoTrajectory(sample.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("final time: " + timer.get());
    System.out.println("expected time: " + trajectory.getTotalTime());
    s_Swerve.setControl(controlSystems.autoDrive(0, 0, 0));
    timer.stop();
    Pose2d pose = s_Swerve.getPose();
    Optional<Pose2d> goal = trajectory.getFinalPose(alliance.get() == DriverStation.Alliance.Red);
    System.out.println("x error: " + (pose.getX() - goal.get().getX()));
    System.out.println("y error: " + (pose.getY() - goal.get().getY()));
    System.out.println("rot error: " + (pose.getRotation().getDegrees() - goal.get().getRotation().getDegrees()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory != null ? timer.hasElapsed(trajectory.getTotalTime()+ 0.2) : true;
  }

  private void followAutoTrajectory(SwerveSample sample){
        Pose2d currPose = s_Swerve.getPose();

        System.out.println("forward velocity: " + sample.vx);
        

      // s_Swerve.applyFieldSpeeds((ChassisSpeeds.fromFieldRelativeSpeeds(sample.vx + xController.calculate(currPose.getX(), sample.x), sample.vy + yController.calculate(currPose.getY(), sample.y), sample.omega + thetaController.calculate(currPose.getRotation().getRadians(), sample.heading),  currPose.getRotation())));
    
    // s_Swerve.applyFieldSpeeds(
    //   new ChassisSpeeds(
    //     sample.vx + xController.calculate(currPose.getX(), sample.x),
    //     sample.vy + yController.calculate(currPose.getY(), sample.y),
    //     sample.omega + thetaController.calculate(currPose.getRotation().getRadians(), sample.heading)
    //   )
    // );

       s_Swerve.setControl(
        controlSystems.autoDrive(
          sample.vx + xController.calculate(currPose.getX(), sample.x),
          sample.vy + yController.calculate(currPose.getY(), sample.y),
          sample.omega + thetaController.calculate(currPose.getRotation().getRadians(), sample.heading)
        )
       );
       System.out.println("x error: " + (currPose.getX() - sample.x));
       System.out.println("y error: " + (currPose.getY() - sample.y));
       System.out.println("rot error: " + Units.radiansToDegrees((currPose.getRotation().getRadians() - sample.heading)));
    
        // .withVelocityX(sample.vx + xController.calculate(currPose.getX(), sample.x))
        // .withVelocityY(sample.vy + yController.calculate(currPose.getY(), sample.y))
        // .withRotationalRate(sample.omega + thetaController.calculate(currPose.getRotation().getRadians(), sample.heading))
        // );
    }

}
