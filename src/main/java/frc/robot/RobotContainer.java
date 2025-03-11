// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
        import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
        import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.IndexCoral;
import frc.robot.commands.EndEffector.OuttakeCoral;
        import frc.robot.commands.CommandFactory.CommandFactory;
        import frc.robot.commands.Funnel.SetFunnelState;
import frc.robot.commands.Slapdown.Pivot.SetPivotState;
import frc.robot.commands.Slapdown.Roller.SetRollerState;
        import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Funnel.FunnelState;
        import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Slapdown;

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

  private ElevatorState reefPoleLevel = ElevatorState.L2; //default reef pole level

  private ElevatorState operatorPoleLevel = ElevatorState.L2;

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
    // driver.start().onTrue(new InstantCommand(()-> intake.zeroPivotPosition()));
    // driver.b().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(-0.5)));
    // driver.a().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(0)));
    // driver.x().onTrue(new InstantCommand(()-> endEffector.setAlgaeSpeed(0.5)));

    // driverDpadUp.onTrue(new InstantCommand(()-> endEffector.setSpeed(0.5)));
   //  driverDpadDown.onTrue(new InstantCommand(()-> endEffector.setSpeed(-0.5)));
    // driverDpadLeft.onTrue(new InstantCommand(()-> endEffector.setSpeed(0)));

    // driver.a().onTrue(new InstantCommand(() -> elevator.zeroPosition()));
    
    // driver.a().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(20)));
    // driver.b().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(-20)));
    // driver.x().onTrue(new InstantCommand(()-> elevator.setTorqueOutput(0)));
    // driver.x().onTrue(new InstantCommand(()-> elevator.setSpeed(0.1)));
    // driver.b().onTrue(new InstantCommand(()-> elevator.setSpeed(-0.1)));
    // driver.a().onTrue(new InstantCommand(()-> elevator.stopPivot()));


    // driver.x().onTrue(new FollowChoreoTrajectory("halfmeter"));

    // driverX.onTrue(new InstantCommand(()-> elevator.setSpeed(0.1)));
    // driverA.onTrue(new InstantCommand(()-> elevator.setSpeed(-0.1)));
    // driverB.onTrue(new InstantCommand(()-> elevator.setSpeed(0)));



     driver.a().onTrue(new SetPivotState(Slapdown.PivotState.UP));
     driver.b().onTrue(new SetPivotState(Slapdown.PivotState.DOWN));


    // driver.a().onTrue(CommandFactory.AutoScoreCoral(reefPoleLevel, ReefPoleSide.LEFT, driver));

    // driver.b().onTrue(new InstantCommand(() -> controlSystem.upKV()));
    // driver.x().onTrue(new InstantCommand(() -> controlSystem.downKV()));

    // driver.x().onTrue(CommandFactory.AutoScoreCoral(reefPoleLevel, ReefPoleSide.LEFT, driver));
    // driver.b().onTrue(CommandFactory.AutoScoreCoral(reefPoleLevel, ReefPoleSide.RIGHT, driver));
    // driver.povDown().onTrue(CommandFactory.SmartAlgaeIntake());
    // operatorLeftBumper.onTrue(new InstantCommand(()-> operatorPoleLevel = operatorPoleLevel.raiseLevel()));

    // final binds (not really)
    driver.back().onTrue(new InstantCommand(() -> drivetrain.resetOdo()));

    // driver.rightBumper().onTrue(new InstantCommand(() -> reefPoleLevel = reefPoleLevel.raiseLevel()));
    // driver.leftBumper().onTrue(new InstantCommand(() -> reefPoleLevel = reefPoleLevel.decreaseLevel()));

    // driver.a().onTrue(new AltSetElevator(reefPoleLevel));
    driver.leftTrigger().onTrue(new SetFunnelState(FunnelState.OFF));
    driver.leftBumper().onTrue(new SetFunnelState(FunnelState.INTAKE));
    driver.rightTrigger().onTrue(new OuttakeCoral());
    driver.rightBumper().onTrue(new IndexCoral());
    driverRightTrigger.onTrue(CommandFactory.ZeroAll());
    //wtf is this why r there two righttriggers

    driver.povDown().onTrue(new SetElevator(ElevatorState.L1));
    driver.povRight().onTrue(new SetElevator(ElevatorState.L2));
    driver.povLeft().onTrue(new SetElevator(ElevatorState.L3));
    driver.povUp().onTrue(new SetElevator(ElevatorState.L4));

    


//    driver.a().onTrue(CommandFactory.SmartAlgaeIntake());
//    driver.b().onTrue(new SetRollerState(RollerState.OUTTAKE));
//    driver.x().onTrue(new SetRollerState(RollerState.OFF));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {
    configureBindings();
  }
}
