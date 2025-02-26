package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;

public class SmartCoralIndex extends Command {
  EndEffector s_EndEffector;

  public SmartCoralIndex() {
    s_EndEffector = EndEffector.getInstance();
    addRequirements(s_EndEffector);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    new SetOuttake(OuttakeState.INDEX);
  }

  @Override
  public boolean isFinished() {
    return s_EndEffector.getBeamResult();
  }
}