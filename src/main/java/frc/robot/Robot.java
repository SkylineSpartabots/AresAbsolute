// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;
import choreo.Choreo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.SensorUtils;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CANCoders;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Autos.AutoCommand;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Autos.ForwardAuto;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.Slapdown.SetPivot;
import frc.robot.commands.Slapdown.ZeroSlapdown;
import frc.robot.commands.SwerveCommands.ForwardLog;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  SendableChooser<Command> chosenAuto = new SendableChooser<Command>();

  SendableChooser<AutoCommand> firstAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> secondAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> thirdAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fourthAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fifthAuto = new SendableChooser<AutoCommand>();

  SendableChooser<Boolean> sourceSide = new SendableChooser<Boolean>();

  private AutoCommand firstSavedChoice;
  private AutoCommand secondSavedChoice;
  private AutoCommand thirdSavedChoice;
  private AutoCommand fourthSavedChoice;
  private AutoCommand fifthSavedChoice;

  private LEDs led;

 
    public Robot() { 
      // oops just realized logging needs to be in the constructor lol
      // facebookdata
      SignalLogger.setPath("/media/sda/ctre-logs/");

      DataLogManager.start();
      Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
      Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
      Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
      Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
      Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
      switch (BuildConstants.DIRTY) {
        case 0:
          Logger.recordMetadata("GitDirty", "All changes committed");
          break;
        case 1:
          Logger.recordMetadata("GitDirty", "Uncomitted changes");
          break;
        default:
          Logger.recordMetadata("GitDirty", "Unknown");
          break;
      }
      // actual logging
      // Automatically switch between sim and real deployment - to run REPLAY you must manually change Constants.deployMode
      if (isReal()) {
        Constants.deployMode = Constants.Mode.REAL;
      } else {
        Constants.deployMode = Constants.Mode.SIM;
      }

      // Set up data receivers & replay source
      switch (Constants.deployMode) {
        case REAL:
          // Running on a real robot, log to a USB stick ("/U/logs")
          System.out.println("Running in REAL mode");
          Logger.addDataReceiver(new WPILOGWriter());
          Logger.addDataReceiver(new NT4Publisher());
          break;

        case SIM:
          // Running a physics simulator, log to NT
          System.out.println("Running in SIM mode");
          Logger.addDataReceiver(new NT4Publisher());
          break;

        case REPLAY:
          // Replaying a log, set up replay source
          System.out.println("Running in REPLAY mode");
          setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
          break;
      }

      // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
      
      if(DriverStation.getAlliance() != null){
        Constants.alliance = DriverStation.getAlliance().get();
      }

      //Init all subsystems
      Vision.getInstance();
      Elevator.getInstance();
      CommandSwerveDrivetrain.getInstance();
      RobotState.getInstance();
      Slapdown.getInstance();
      DriveControlSystems.getInstance();
      Funnel.getInstance();
      Slapdown.getInstance();
      EndEffector.getInstance();
      RobotContainer.getInstance();
      CANCoders.getInstance();
      Climb.getInstance();
      // LEDs.getInstance();
      // led = LEDs.getInstance();
      CanBridge.runTCP();
    }

  @Override
  public void robotInit() {
    firstAuto.addOption(AutoCommand.Test1().name, AutoCommand.Test1());
    firstAuto.addOption(AutoCommand.Test2().name, AutoCommand.Test2());
    firstAuto.addOption(AutoCommand.meter1().name, AutoCommand.meter1());
    firstAuto.addOption(AutoCommand.meter2().name, AutoCommand.meter2());
    firstAuto.addOption(AutoCommand.meter3().name, AutoCommand.meter3());
    firstAuto.addOption(AutoCommand.backandforth().name, AutoCommand.backandforth());
    firstAuto.addOption(AutoCommand.B1R3().name, AutoCommand.B1R3());
    firstAuto.addOption(AutoCommand.B2R8().name, AutoCommand.B2R8());

    sourceSide.setDefaultOption("left source", false);
    sourceSide.addOption("right source", true);
    
    SmartDashboard.putString("current path", "none");
    chosenAuto.addOption("forward + dealgae back", Autos.forwardDealgaeBack());
    chosenAuto.addOption("forward + dealgae left", Autos.forwardDealgaeLeft());
    chosenAuto.addOption("forward + dealgae right", Autos.forwardDealgaeRight());
    chosenAuto.addOption("1 + 1 left", Autos.twoCoralLeft());
    chosenAuto.addOption("1 + 1 right", Autos.twoCoralRight());
    chosenAuto.addOption("1 + 2 left", Autos.threeCoralLeft());
    chosenAuto.addOption("1 + 2 right", Autos.threeCoralRight());
    // firstAuto.addOption(AutoCommand.halfmeter().name, AutoCommand.halfmeter());
    // AutoCommand.loadAutos(); 
    SmartDashboard.putData("source side", sourceSide);
    SmartDashboard.putData("first auto", firstAuto);
    SmartDashboard.putData("autochoices", chosenAuto);

    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter()); // should be savig to usb
      Logger.addDataReceiver(new NT4Publisher());
    }
    else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    // Logger.start();

    //start the logger here
    SmartDashboard.putString("Selected Pole", "POLE_A");
    SmartDashboard.putString("Selected Pole Level", ElevatorState.L1.name());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    // if(firstAuto.getSelected() != firstSavedChoice){ //Note: might be able to use the onchange() method in sendable chooser
    //   firstSavedChoice = firstAuto.getSelected();
    //   // m_autonomousCommand.addCommands(firstSavedChoice.getCommand());
    //   updateSecondAuto();
    // }
    // if(secondAuto.getSelected() != secondSavedChoice){
    //   secondSavedChoice = secondAuto.getSelected();
    //   updateThirdAuto();
    //   // m_autonomousCommand.addCommands(secondSavedChoice.getCommand());
    // }
    // if(thirdAuto.getSelected() != thirdSavedChoice){
    //   thirdSavedChoice = thirdAuto.getSelected();
    //   // m_autonomousCommand.addCommands(thirdSavedChoice.getCommand());
    // }
    // CANCoders.getInstance().getOffsets();
    // led.updateDisabled();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Constants.alliance = DriverStation.getAlliance().get();
    // m_autonomousCommand = new SequentialCommandGroup();
    // new SequentialCommandGroup(
    //   new ZeroSlapdown(),
    //   new ZeroElevator(),
    //   new ForwardAuto()
    // ).schedule();
    Constants.alliance = DriverStation.getAlliance().get();
    m_autonomousCommand = chosenAuto.getSelected();
    RobotState.getInstance().setSourceValue(sourceSide.getSelected());
    new SequentialCommandGroup(
      // new ZeroSlapdown(),
      new ZeroElevator(),
      m_autonomousCommand

    ).schedule();
    
    
    // if(firstSavedChoice != null) m_autonomousCommand.addCommands(firstSavedChoice.getCommand());
    
    // if(secondSavedChoice != null) m_autonomousCommand.addCommands(secondSavedChoice.getCommand());
    // if(thirdSavedChoice != null) m_autonomousCommand.addCommands(thirdSavedChoice.getCommand());
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotState.getInstance().setSourceValue(sourceSide.getSelected());
    // new ZeroSlapdown().schedule();
    Constants.alliance = DriverStation.getAlliance().get();

    // new ForwardLog().schedule();
    
  }

  @Override
  public void teleopPeriodic() {
    // led.updateTeleop();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void updateSecondAuto(){
    if(secondAuto != null){
      secondAuto.close();
    }
    secondAuto = new SendableChooser<AutoCommand>();
    firstSavedChoice = firstAuto.getSelected();
    AutoCommand.clearContinuations();
    AutoCommand.fillAutosList(firstSavedChoice);
      for(int i = 0; i < AutoCommand.getPotentialContinuations().size(); i++){
        secondAuto.addOption(AutoCommand.getPotentialContinuations().get(i).name, 
        AutoCommand.getPotentialContinuations().get(i));
      }
      SmartDashboard.putData("second auto", secondAuto);
  }

  private void updateThirdAuto(){
    if(thirdAuto != null){
      thirdAuto.close();
    }
    thirdAuto = new SendableChooser<AutoCommand>();
    secondSavedChoice = secondAuto.getSelected();
    AutoCommand.clearContinuations();
    AutoCommand.fillAutosList(secondSavedChoice);
      for(int i = 0; i < AutoCommand.getPotentialContinuations().size(); i++){
        thirdAuto.addOption(AutoCommand.getPotentialContinuations().get(i).name, 
        AutoCommand.getPotentialContinuations().get(i));
      }
      SmartDashboard.putData("third auto", thirdAuto);
  }

}

