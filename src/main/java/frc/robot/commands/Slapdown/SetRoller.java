// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown.RollerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRoller extends Command {
  private Slapdown s_Slapdown;
  private RollerState state;
  Timer timer = new Timer();
  
  public SetRoller(RollerState state) {
    s_Slapdown = Slapdown.getInstance();
    this.state = state;
    addRequirements(s_Slapdown);
  }

  @Override
  public void initialize() {
    timer.start();
    s_Slapdown.setRollerSpeed(state.getRollerSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if(state == RollerState.OUTTAKE){
      s_Slapdown.setRollerSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == RollerState.OUTTAKE) {
      if(timer.hasElapsed(0.75)) {
        return true;
      } else return false;
    } else {
        return true;
    }
  }
}
