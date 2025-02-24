package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;

public class setOuttake extends Command {
  EndEffector s_EndEffector;
  Timer timer = new Timer();
  boolean finished = false;
  OuttakeState state;

  public setOuttake(OuttakeState state) {
    s_EndEffector = EndEffector.getInstance();
    addRequirements(s_EndEffector);
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    s_EndEffector.setOuttakeSpeed(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_EndEffector.setOuttakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == OuttakeState.INDEX && timer.hasElapsed(0.09)
     || (state == OuttakeState.SCOREMID && timer.hasElapsed(0.5)));
  }
}