// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FFChoreoTrajectory extends Command {
  private Trajectory trajectory;
  private final CommandSwerveDrivetrain s_Swerve;
  private Optional<DriverStation.Alliance> alliance;
  private Optional<Pose2d> startPose;
  private Timer timer;
  private DriveControlSystems controlSystems;
  private PIDController thetaController = new PIDController(0.4, 0, 0);
  public FFChoreoTrajectory(String name) {
    if (Choreo.loadTrajectory(name).isPresent()) {
      trajectory = Choreo.loadTrajectory(name).get();
    }
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    alliance = DriverStation.getAlliance();
    timer = new Timer();
    controlSystems = DriveControlSystems.getInstance();
    
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (trajectory != null){
      startPose = trajectory.getInitialPose(alliance.get() == DriverStation.Alliance.Red);
      s_Swerve.resetOdo(startPose.get());
    }
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
    s_Swerve.setControl(controlSystems.autoDrive(0, 0, 0));
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
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
          sample.vx,
          sample.vy,
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
