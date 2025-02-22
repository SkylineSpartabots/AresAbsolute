// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import frc.robot.Constants;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SeekPivotState extends Command {
  private Slapdown s_Slapdown;
  private double angleSetpoint;
  private double speed;
  private PivotState state;
  private PIDController controller = new PIDController(2.8, 0, 0);
  
  public SeekPivotState(PivotState state) {
    s_Slapdown = Slapdown.getInstance();
    angleSetpoint = state.getPosition();
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state != PivotState.DOWN){
      s_Slapdown.setPivotSpeed(-0.35);
    } else{
      s_Slapdown.setPivotSpeed(0.3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Slapdown.setPivotVoltage(controller.calculate(s_Slapdown.getPosition(), angleSetpoint));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Slapdown.brakePivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(s_Slapdown.getPosition() - angleSetpoint) < 0.1 || s_Slapdown.getPivotCurrent() > Constants.intakePivotCurrentThreshold;
  }
}
