// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetOuttake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardAuto extends Command {
  
  private CommandSwerveDrivetrain dt;
  private DriveControlSystems controlSystems;
  double timebeforeextension = 0.3;
  double drivetime = 8;

  boolean extended = false;

  Timer timer;
  private Alliance alliance;
  private PIDController thetaController = new PIDController(0.5, 0, 0);

  public ForwardAuto() {
    dt = CommandSwerveDrivetrain.getInstance();
    controlSystems = DriveControlSystems.getInstance();
    timer = new Timer();


    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance().get();
    timer.reset();
    timer.start();
    dt.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    dt.setControl(controlSystems.robotCentricDrive(0.8, 0, 0));
    
    
    SmartDashboard.putBoolean("auto running", true);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = dt.getPose();
    dt.setControl(controlSystems.robotCentricDrive(0.8, 0, thetaController.calculate(currPose.getRotation().getRadians(), 0)));
    }
    
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("time: " + timer.get());
    timer.stop();
    dt.setControl(
      controlSystems.autoDrive(0, 0, 0)
    );
    new SetOuttake(OuttakeState.SCORE).schedule();
    SmartDashboard.putBoolean("auto running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(drivetime);
  }
}
