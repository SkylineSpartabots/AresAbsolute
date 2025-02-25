// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown;

import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;

import java.lang.Thread.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPivot extends Command {
  private Slapdown s_Slapdown;
  private double angleSetpoint;
  private PivotState state;
  private PIDController controller = new PIDController(4.1, 2.3, 0.3);

  public SetPivot(PivotState state) {
    s_Slapdown = Slapdown.getInstance();
    angleSetpoint = state.getPosition();
    this.state = state;
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    s_Slapdown.setPivotVoltage(controller.calculate(s_Slapdown.getPosition(), angleSetpoint));
  }

  @Override
  public void end(boolean interrupted) {
    s_Slapdown.setPivotVoltage(0);

    if(state != PivotState.DOWN)
      s_Slapdown.brakePivot(state);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(s_Slapdown.getPosition() - angleSetpoint) < 0.1;
  }
}
