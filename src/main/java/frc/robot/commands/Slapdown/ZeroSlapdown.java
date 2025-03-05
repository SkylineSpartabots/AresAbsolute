// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroSlapdown extends Command {
  private Slapdown s_Intake;
  private Timer timer;

  public ZeroSlapdown(){
    s_Intake = Slapdown.getInstance();
    addRequirements(s_Intake);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Intake.setPivotSpeed(-0.1);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.setPivotPosition(PivotState.UP);;
    if(!interrupted){
      s_Intake.resetPivotPosition();
    }
    timer.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(s_Intake.getPivotVelocity()) < 0.2 && timer.hasElapsed(0.2);
  }
}
