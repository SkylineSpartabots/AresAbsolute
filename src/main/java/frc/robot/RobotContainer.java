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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.commands.AltSetElevator;
import frc.robot.commands.CancelableCommand;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetIntakePivot;
import frc.robot.commands.SetIntakeRoller;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.CommandFactory.CommandFactory;
import frc.robot.commands.CommandFactory.CommandFactory.*;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleLevel;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Intake.PivotState;
import frc.robot.Subsystems.Intake.RollerState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Intake;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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

  private DriveControlSystems controlSystem  = DriveControlSystems.getInstance();

  private ReefPoleLevel reefPoleLevel = ReefPoleLevel.L2; //default reef pole level

  //instances
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // Drivetrain
  private final Intake intake = Intake.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();

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
    // driver.a().onTrue(new AltSetElevator(ElevatorState.GROUND));
    // driver.b().onTrue(new AltSetElevator(ElevatorState.L1));
    // driver.y().onTrue(new AltSetElevator(ElevatorState.L3));
    // driver.x().onTrue(new AltSetElevator(ElevatorState.L2));



    driver.rightBumper().onTrue(new InstantCommand( () -> reefPoleLevel = reefPoleLevel.raiseLevel()));
    driver.leftBumper().onTrue(new InstantCommand(() -> reefPoleLevel = reefPoleLevel.decreaseLevel()));

    driverBack.onTrue(new InstantCommand(() -> drivetrain.resetOdo()));

    // driver.a().onTrue(CommandFactory.AutoScoreCoral(reefPoleLevel, ReefPoleSide.LEFT, driver));

    // driver.start().onTrue(new ZeroElevator());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {
    configureBindings();
  }
}
