// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.FollowChoreoTrajectory;

/** Add your docs here. */
public class Autos {
    private RobotContainer m_robotContainer;

    private final Vision vision = Vision.getInstance();

    private static CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private static RobotState robotState = RobotState.getInstance();
    // private static final PIDController thetaController = new PIDController(0, 0, 0); //tune?
    // private static final PIDController xController = new PIDController(0, 0, 0);
    // private static final PIDController yController = new PIDController(0, 0, 0);

  public static Command Test1() {
    return Commands.waitSeconds(1);
  }

  public static Command WaitSeconds(double seconds){
    return Commands.waitSeconds(seconds);
  }

  public static Command Test2() {
    return Commands.waitSeconds(0);
  }

  public static Command Test3() {
    return Commands.waitSeconds(0);
  }
  
  public static Command Test4() {
    return Commands.waitSeconds(0);
  }

  public static Command meter() {
    return new FollowChoreoTrajectory("1meter");
  }

  public static Command meter2(){
    return new FollowChoreoTrajectory("2meters");
  }

  public static Command meter3(){
    return new FollowChoreoTrajectory("3meters");
  }

  public static Command meter4(){
    return new FollowChoreoTrajectory("4meters");
  }

  public static Command halfmeter(){
    return new FollowChoreoTrajectory("halfmeter");
  }


  // public static Command meterForwardTest(){
  //   return new SequentialCommandGroup(
  //     new FollowChoreoTrajectory("meterForwardTest")
  //   );
  // }
}
