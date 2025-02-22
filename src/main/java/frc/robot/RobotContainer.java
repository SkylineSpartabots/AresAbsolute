// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.time.Instant;

import org.opencv.core.Point;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.commands.CancelableCommand;
import frc.robot.commands.VisionKalmanTest;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.CommandFactory.CommandFactory;
import frc.robot.commands.CommandFactory.CommandFactory.*;
import frc.robot.commands.Elevator.AltSetElevator;
import frc.robot.commands.Elevator.ElevatorTest;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.Funnel.SetFunnel;
import frc.robot.commands.Pivot.SetSlapdownPivot;
import frc.robot.commands.Pivot.SmartIntake;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleLevel;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Slapdown;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {

  private static RobotContainer container;

  public static RobotContainer getInstance(){//so i can grab controller values lol
      if(container == null){
          container = new RobotContainer();
      }
      return container;
  }


  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController driver = new CommandXboxController(0); // Driver joystick
  public final CommandXboxController operator = new CommandXboxController(1);

  private DriveControlSystems controlSystem  = DriveControlSystems.getInstance();

  private ReefPoleLevel reefPoleLevel = ReefPoleLevel.L2; //default reef pole level

  private ReefPoleLevel operatorPoleLevel = ReefPoleLevel.L2;

  //instances
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // Drivetrain
  private final Slapdown intake = Slapdown.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final Funnel funnel = Funnel.getInstance();

  /* Driver Buttons */
  private final Trigger driverBack = driver.back();
  private final Trigger driverStart = driver.start();
  private final Trigger driverA = driver.a();
  private final Trigger driverB = driver.b();
  private final Trigger driverX = driver.x();
  private final Trigger driverY = driver.y();
  private final Trigger driverRightBumper = driver.rightBumper();
  private final Trigger driverLeftBumper = driver.rightBumper();
  private final Trigger driverLeftTrigger = driver.leftTrigger();
  private final Trigger driverRightTrigger = driver.rightTrigger();
  private final Trigger driverDpadUp = driver.povUp();
  private final Trigger driverDpadDown = driver.povDown();
  private final Trigger driverDpadLeft = driver.povLeft();
  private final Trigger driverDpadRight = driver.povRight();

  private final Trigger operatorBack = operator.back();
  private final Trigger operatorStart = operator.start();
  private final Trigger operatorA = operator.a();
  private final Trigger operatorB = operator.b();
  private final Trigger operatorX = operator.x();
  private final Trigger operatorY = operator.y();
  private final Trigger operatorRightBumper = operator.rightBumper();
  private final Trigger operatorLeftBumper = operator.rightBumper();
  private final Trigger operatorLeftTrigger = operator.leftTrigger();
  private final Trigger operatorRightTrigger = operator.rightTrigger();
  private final Trigger operatorDpadUp = operator.povUp();
  private final Trigger operatorDpadDown = operator.povDown();
  private final Trigger operatorDpadLeft = operator.povLeft();
  private final Trigger operatorDpadRight = operator.povRight();

  public CommandXboxController getDriverController(){
      return driver;
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> controlSystem.drive(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()) // Drive counterclockwise with negative X (left)
    ));
    //bindings
    
    driver.leftTrigger().onTrue(CommandFactory.OffEverything());
    // driver.a().onTrue(new InstantCommand(()->intake.testUnbrake()));
    // driver.b().onTrue(new InstantCommand(()->intake.testBrake()));

    // driver.a().onTrue(new SetIntakePivot(PivotState.UP));
    // driver.b().onTrue(new SetIntakePivot(PivotState.DOWN));
    // driver.x().onTrue(CommandFactory.Intake());
    // driver.y().onTrue(CommandFactory.Lift());
    // driverBack.onTrue(new InstantCommand(()-> intake.stopPivot()));

    // driverDpadRight.onTrue(new SmartIntake());

    // driverDpadLeft.onTrue(new InstantCommand(()-> intake.setRollerSpeed(RollerState.INTAKE.getRollerSpeed())));
    // driverDpadUp.onTrue(new InstantCommand(()->intake.setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed())));
    // driverDpadDown.onTrue(new InstantCommand(()->intake.brakeRoller()));
    
    // driver.y().onTrue(new InstantCommand(() -> intake.stopPivot()));
    // driver.start().onTrue(new InstantCommand(()-> intake.resetPivotPosition()));
    // driver.b().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(-0.5)));
    // driver.a().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(0)));
    // driver.x().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(0.5)));

    // driverDpadUp.onTrue(new InstantCommand(()-> endEffector.setSpeed(0.5)));
   //  driverDpadDown.onTrue(new InstantCommand(()-> endEffector.setSpeed(-0.5)));
    // driverDpadLeft.onTrue(new InstantCommand(()-> endEffector.setSpeed(0)));

    // driver.a().onTrue(new InstantCommand(() -> elevator.zeroPosition()));
    driver.a().onTrue(new AltSetElevator(ElevatorState.L4));
    driver.povUp().onTrue(new AltSetElevator(ElevatorState.L1));
    driver.y().onTrue(new AltSetElevator(ElevatorState.L3));
    driver.x().onTrue(new AltSetElevator(ElevatorState.L2));
    driverB.onTrue(new AltSetElevator(ElevatorState.GROUND));
    // driver.a().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(20)));
    // driver.b().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(-20)));
    // driver.x().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(0)));
    // driver.x().onTrue(new InstantCommand(()-> elevator.setSpeed(0.1)));
    // driver.b().onTrue(new InstantCommand(()-> elevator.setSpeed(-0.1)));
    // driver.a().onTrue(new InstantCommand(()-> elevator.stop()));


    // driver.x().onTrue(new FollowChoreoTrajectory("halfmeter"));

    // driverX.onTrue(new InstantCommand(()-> elevator.setSpeed(1)));
    // driverA.onTrue(new InstantCommand(()-> elevator.setSpeed(-1)));
    // driverB.onTrue(new InstantCommand(()-> elevator.setSpeed(0)));
    driverStart.onTrue(new ZeroElevator());

    driver.rightBumper().onTrue(new InstantCommand(() -> reefPoleLevel = reefPoleLevel.raiseLevel()));
    driver.leftBumper().onTrue(new InstantCommand(() -> reefPoleLevel = reefPoleLevel.decreaseLevel()));

    // driver.a().onTrue(new SetSlapdownPivot(PivotState.UP));
    // driver.b().onTrue(new SetSlapdownPivot(PivotState.DOWN));
    // driver.x().onTrue(CommandFactory.smartAlgeaIntake());

    driver.back().onTrue(new InstantCommand(() -> drivetrain.resetOdo()));

    // driver.a().onTrue(CommandFactory.AutoScoreCoral(reefPoleLevel, ReefPoleSide.LEFT, driver));

    // driver.b().onTrue(new InstantCommand(() -> controlSystem.upKV()));
    // driver.x().onTrue(new InstantCommand(() -> controlSystem.downKV()));
    // driver.y().onTrue(new ZeroElevator());
    // driver.a().onTrue(new InstantCommand(() -> funnel.setSpeed(0.5)));
    // driver.b().onTrue(new SetFunnel(FunnelState.OFF));

    // operatorLeftBumper.onTrue(new InstantCommand(()-> operatorPoleLevel = operatorPoleLevel.raiseLevel()));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {
    configureBindings();
  }
}
