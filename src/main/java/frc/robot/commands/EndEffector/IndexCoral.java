package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.EndEffector;

public class IndexCoral extends Command {
    EndEffector s_EndEffector;
    Timer timer = new Timer();
    double initialPosition;

    public IndexCoral() {
        s_EndEffector = EndEffector.getInstance();
        addRequirements(s_EndEffector);
    }

    @Override
    public void initialize() {
        timer.restart();
        s_EndEffector.setCoralSpeed(EndEffector.OuttakeState.INDEX.getSpeed());
        initialPosition = s_EndEffector.getCoralPosition();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_EndEffector.setCoralSpeed(0);
        System.out.println("IndexCoral Ended");
    }

    @Override
    public boolean isFinished() {
        return initialPosition + s_EndEffector.getCoralPosition() >= 0.5 || timer.hasElapsed(0.5); //TODO tune this position and timer values. How far/long to index
    }
}