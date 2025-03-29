package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;

public class SetOuttake extends Command {
  EndEffector s_EndEffector;
  Timer timer = new Timer();
  boolean finished = false;
  OuttakeState state;
  Elevator elevator;

  public SetOuttake(OuttakeState state) {
    s_EndEffector = EndEffector.getInstance();
    addRequirements(s_EndEffector);
    elevator = Elevator.getInstance();
    this.state = state;
  }

  public SetOuttake(Supplier<ElevatorState> state) {
    s_EndEffector = EndEffector.getInstance();
    addRequirements(s_EndEffector);

    this.state = OuttakeState.SCORE;
  } 

  @Override
  public void initialize() {
    timer.restart();
    
  
      s_EndEffector.setOuttakeSpeed(state);
    

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    s_EndEffector.setOuttakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return (state == OuttakeState.INDEX && timer.hasElapsed(0.12)
     || (timer.hasElapsed(0.4)));
  }
}