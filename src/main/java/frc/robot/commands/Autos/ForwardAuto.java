// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
  double drivetime = 2.4;

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
    if(alliance == Alliance.Blue){
      dt.setControl(controlSystems.autoDrive(-1, 0,  0));
    }else {
      dt.setControl(controlSystems.autoDrive(1, 0,  0));
    }
 
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = dt.getPose();
    if(alliance == Alliance.Blue){
      dt.setControl(
      controlSystems.autoDrive(
        -1, 0, thetaController.calculate(currPose.getRotation().getRadians(), Math.PI)
      )
    );
    }else {
      dt.setControl(
      controlSystems.autoDrive(
        1, 0, thetaController.calculate(currPose.getRotation().getRadians(), -Math.PI)
      )
    );
    }
    
    if(timer.hasElapsed(timebeforeextension) && !extended){
      new SequentialCommandGroup(
        new SetElevator(() -> ElevatorState.L4),
        Commands.waitSeconds(1.6),
        new SetOuttake(OuttakeState.SCORE)
      ).schedule();
      extended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("time: " + timer.get());
    timer.stop();
    dt.setControl(
      controlSystems.autoDrive(0, 0, 0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(drivetime);
  }
}
