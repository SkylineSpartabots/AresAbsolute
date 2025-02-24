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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SetOuttake(OuttakeState.INDEX);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_EndEffector.getLaserMeasurement().distance_mm < 3;
  }
}