// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown.RollerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command {
  private Slapdown s_Slapdown;
  Timer timer = new Timer();
  
  public SmartIntake() {
    s_Slapdown = Slapdown.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    s_Slapdown.setRollerSpeed(RollerState.INTAKE.getRollerSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Slapdown.brakeRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //the timer should ensure we dont finish the command early as the motor overcomes static friction
    return s_Slapdown.getResistiveCurrent() > Constants.intakePivotCurrentThreshold && timer.hasElapsed(0.1);
  }
}
