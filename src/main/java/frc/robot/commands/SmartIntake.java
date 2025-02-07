// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.PivotState;
import frc.robot.Subsystems.Intake.RollerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command {
  private Intake s_Intake;
  private double angleSetpoint;
  private PIDController controller = new PIDController(1.9, 0, 0);
  public SmartIntake() {
    s_Intake = Intake.getInstance();
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Intake.setPivotPosition(PivotState.DOWN);
    s_Intake.setRollerSpeed(RollerState.INTAKE.getRollerSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SeekPivotState(PivotState.UP).schedule();
    s_Intake.brakeRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Intake.getRollerCurrent() > Constants.intakePivotCurrentThreshold;
  }
}
