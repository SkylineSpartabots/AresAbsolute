package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.commands.CommandFactory;

public class AutoShootCoral extends Command {
    private EndEffector s_EndEffector;
    Timer timer = new Timer();

    public AutoShootCoral() {
        s_EndEffector = EndEffector.getInstance();

        addRequirements(s_EndEffector);
    }

    public void initialize() {
        timer.restart();
        s_EndEffector.setOuttakeSpeed(OuttakeState.SCORE);
    }

    public void end() {
        s_EndEffector.setOuttakeSpeed(0);
        CommandFactory.IntakePath(() -> RobotState.getInstance().getSourceValue()).schedule();
    }

    public boolean isFinished() {
        return timer.hasElapsed(0.5); //need the elevator to not be wiggling till test this #
    }

}
