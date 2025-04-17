// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CANCoders;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.RunClimb;
import frc.robot.commands.Autos.FFChoreoTrajectory;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Elevator.AdjustElevator;
import frc.robot.commands.Elevator.ManualElevatorZero;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.EndEffector.SetOuttake;
import frc.robot.commands.EndEffector.SmartCoralIntake;
import frc.robot.commands.Funnel.SetFunnel;
import frc.robot.commands.Slapdown.SetPivot;
import frc.robot.commands.Slapdown.SetRoller;
import frc.robot.commands.Slapdown.ZeroSlapdown;
import frc.robot.commands.SwerveCommands.SlowDrive;
import frc.robot.commands.TeleopAutomation.PoleAlign;

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

  //instances
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // Drivetrain
  private final RobotState robotstate = RobotState.getInstance(); // Drivetrain
  private final Slapdown intake = Slapdown.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final Funnel funnel = Funnel.getInstance();
  private final Climb climb = Climb.getInstance();
  private final CANCoders cancoders = CANCoders.getInstance();

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

    //SysID
        // driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // driver.a().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kForward)); // Start SysId Dynamic test in forward direction
        // driver.b().whileTrue(Elevator.getInstance().sysIdDynamic(Direction.kReverse)); // Start SysId Dynamic test in backward direction
        // driver.x().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kForward)); // Start SysId Quasistatic test in forward direction
        // driver.y().whileTrue(Elevator.getInstance().sysIdQuasistatic(Direction.kReverse)); // Start SysId Quasistatic test in backward direction
        
  
              
    // ----------====# Active binding ====----------

    driverDpadLeft.onTrue(CommandFactory.Dealgaeify(ElevatorState.A1));
    driverDpadRight.onTrue(CommandFactory.Dealgaeify(ElevatorState.A2));
    driverDpadUp.whileTrue(new RunClimb(-0.95));
    driverDpadDown.whileTrue(new RunClimb(0.95));
  
    driver.leftBumper().onTrue(new InstantCommand(() -> robotstate.lowerPoleLevel()));
    driver.rightBumper().onTrue(new InstantCommand(() -> robotstate.raisePoleLevel()));

    driverLeftTrigger.whileTrue(new SlowDrive()); // :(

    driverRightTrigger.onTrue(new InstantCommand(()->endEffector.setOuttakeSpeed(-0.2769))); //scoring on L2-L4
    driverRightTrigger.onFalse(new InstantCommand(()->endEffector.setOuttakeSpeed(0)));
    driver.start().onTrue(new InstantCommand(()->endEffector.setOuttakeSpeed(-0.56))); //theoretical outtake speed for scoring L1
    driver.start().onFalse(new InstantCommand(()->endEffector.setOuttakeSpeed(0)));

    driver.back().onTrue(CommandFactory.EjectFunnel());

    driver.a().onTrue(new SetElevator(() -> robotstate.getSelectedElevatorLevel()));
    driver.x().onTrue(CommandFactory.FlexibleIntake());
    driver.y().onTrue(CommandFactory.BeginAutomationRoutine(driver));

    // ----------====# Operator bindings #====----------
    operator.start().onTrue(new ZeroElevator());
    
    operator.povLeft().onTrue(new InstantCommand(() -> robotstate.setSourceValue(false)));
    operator.povRight().onTrue(new InstantCommand(() -> robotstate.setSourceValue(true)));

    operator.b().onTrue(new InstantCommand(() -> robotstate.setAutoOnIntake(false)));
    operator.a().onTrue(new InstantCommand(() -> robotstate.setAutoOnIntake(true)));

    operator.x().onTrue(new InstantCommand(()->robotstate.toggleSource()));
    
    operator.rightBumper().onTrue(new InstantCommand(() -> robotstate.navigateReefPoleUp()));
    operator.leftBumper().onTrue(new InstantCommand(() -> robotstate.navigateReefPoleDown()));
    
    operatorDpadUp.whileTrue(new AdjustElevator(0.2));
    operatorDpadDown.whileTrue(new AdjustElevator(-0.2));

    operator.leftTrigger().onTrue(new InstantCommand(()-> endEffector.setOuttakeSpeed(0.4)));
    operator.leftTrigger().onFalse(new InstantCommand(()->endEffector.setOuttakeSpeed(0)));
    operator.rightTrigger().onTrue(new InstantCommand(()-> endEffector.setOuttakeSpeed(-0.3)));
    operator.rightTrigger().onFalse(new InstantCommand(()->endEffector.setOuttakeSpeed(0)));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {

    configureBindings();
  }
}
