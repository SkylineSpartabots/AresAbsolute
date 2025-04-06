package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.EndEffector;

public class CheckBeamBreak extends Command {

    EndEffector s_EndEffector;
    
    public CheckBeamBreak() {
        s_EndEffector = EndEffector.getInstance();
    }

  public void initialize() {
    if(s_EndEffector.getBeamResult()) { //if we already have a coral SKIP this command

    }
  }

    public void end() {
      // CommandFactory.AutoPathScoreCoralWithDefender(null, null).schedule();
    }
  
    public boolean isFinished() {
      return true;
    }
}