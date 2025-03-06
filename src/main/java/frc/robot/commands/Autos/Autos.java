// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetOuttake;

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

  public static Command B1R3(){
    return new  SequentialCommandGroup(
      new FollowChoreoTrajectory("B1R3"),
      Commands.waitSeconds(0.1),
      new SetElevator(() -> ElevatorState.L4),
      Commands.waitSeconds(0.1),
      new GoFlush(),
      Commands.waitSeconds(0.4),
      new SetOuttake(OuttakeState.SCORE)
    );

  }

  public static Command B2R8(){
    return new FollowChoreoTrajectory("B2R8");
  }

  public static Command S1R1(){
    return new FollowChoreoTrajectory("S1R1");
  }

  public static Command S1R2(){
    return new FollowChoreoTrajectory("S1R2");
  }

  public static Command R8S2(){
    return new FollowChoreoTrajectory("R8S2");
  }
  public static Command R3S1(){
    return new FollowChoreoTrajectory("R3S1");
  }
  public static Command S2R9(){
    return new FollowChoreoTrajectory("S2R9");
  }

  public static Command S2R10(){
    return new FollowChoreoTrajectory("S2R10");
  }

  public static Command S2R11(){
    return new FollowChoreoTrajectory("S2R11");
  }
  public static Command S2R12(){
    return new FollowChoreoTrajectory("S2R12");
  }

  public static Command R10S2(){
    return new FollowChoreoTrajectory("R10S2");
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
    return new FollowChoreoTrajectory("2meter");
  }

  public static Command meter3(){
    return new FollowChoreoTrajectory("3meter");
  }

  public static Command backandforth(){
    return new FollowChoreoTrajectory("backandforth");
  }


  // public static Command halfmeter(){
  //   return new FollowChoreoTrajectory("halfmeter");
  // }


  // public static Command meterForwardTest(){
  //   return new SequentialCommandGroup(
  //     new FollowChoreoTrajectory("meterForwardTest")
  //   );
  // }
}
