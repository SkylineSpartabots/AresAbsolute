// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardLog extends Command {
  private CommandSwerveDrivetrain dt;
  private DriveControlSystems controlSystems;
  private Timer timer;
  public ForwardLog() {
    dt = CommandSwerveDrivetrain.getInstance();
    controlSystems = DriveControlSystems.getInstance();
    timer = new Timer();


    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.setControl(controlSystems.autoDrive(1, 0, 0));
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setControl(controlSystems.autoDrive(0, 0, 0));
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
