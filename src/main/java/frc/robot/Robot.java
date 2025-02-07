// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.SensorUtils;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;

public class Robot extends LoggedRobot {
  private SequentialCommandGroup m_autonomousCommand = new SequentialCommandGroup();
  SendableChooser<AutoCommand> firstAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> secondAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> thirdAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fourthAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fifthAuto = new SendableChooser<AutoCommand>();

  private AutoCommand firstSavedChoice;
  private AutoCommand secondSavedChoice;
  private AutoCommand thirdSavedChoice;
  private AutoCommand fourthSavedChoice;
  private AutoCommand fifthSavedChoice;

  private RobotContainer m_robotContainer;

  private final Vision vision;

  private CommandSwerveDrivetrain drivetrain;
  private Elevator elevator;
  private RobotState robotState;
  private Intake intake;
 
    public Robot() { 
      // oops just realized logging needs to be in the constructor lol
      // metadata
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
      
      elevator = Elevator.getInstance();
      drivetrain = CommandSwerveDrivetrain.getInstance();
      robotState = RobotState.getInstance();
      vision = Vision.getInstance();
      intake = Intake.getInstance();
    }

  @Override
  public void robotInit() {
    firstAuto.addOption(AutoCommand.Test1().name, AutoCommand.Test1());
    firstAuto.addOption(AutoCommand.Test2().name, AutoCommand.Test2());
    firstAuto.addOption(AutoCommand.meter1().name, AutoCommand.meter1());
    firstAuto.addOption(AutoCommand.meter2().name, AutoCommand.meter2());
    firstAuto.addOption(AutoCommand.meter3().name, AutoCommand.meter3());
    firstAuto.addOption(AutoCommand.meter4().name, AutoCommand.meter4());
    firstAuto.addOption(AutoCommand.halfmeter().name, AutoCommand.halfmeter());
    // AutoCommand.loadAutos(); TODO ethan fix this
    SmartDashboard.putData("first auto", firstAuto);

    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter()); // should be savig to usb
      Logger.addDataReceiver(new NT4Publisher());
    }
    else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();

    //start the logger here
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(firstAuto.getSelected() != firstSavedChoice){ //Note: might be able to use the onchange() method in sendable chooser
      firstSavedChoice = firstAuto.getSelected();
      // m_autonomousCommand.addCommands(firstSavedChoice.getCommand());
      updateSecondAuto();
    }
    if(secondAuto.getSelected() != secondSavedChoice){
      secondSavedChoice = secondAuto.getSelected();
      // m_autonomousCommand.addCommands(secondSavedChoice.getCommand());
    }
    if(thirdAuto.getSelected() != thirdSavedChoice){
      thirdSavedChoice = thirdAuto.getSelected();
      // m_autonomousCommand.addCommands(thirdSavedChoice.getCommand());
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if(firstSavedChoice != null) m_autonomousCommand.addCommands(firstSavedChoice.getCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

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

}

