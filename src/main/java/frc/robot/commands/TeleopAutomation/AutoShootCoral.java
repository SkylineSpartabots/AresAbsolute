package frc.robot.commands.TeleopAutomation;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.commands.CommandFactory;

public class AutoShootCoral extends Command {
    private EndEffector s_EndEffector;
    Timer timer;
    private CommandXboxController controller;

    public  AutoShootCoral(CommandXboxController controller) {
        s_EndEffector = EndEffector.getInstance();
        this.controller = controller;
        timer = new Timer();
        addRequirements(s_EndEffector);
    }

    @Override
    public void initialize() {
        timer.restart();
        s_EndEffector.setOuttakeSpeed(OuttakeState.SCORE);
    }

    @Override
    public void end(boolean interrupted) {
        s_EndEffector.setOuttakeSpeed(0);
        CommandFactory.IntakePath(controller).schedule();

    }

    public boolean isFinished() {
        return timer.hasElapsed(0.5); //need the elevator to not be wiggling till test this #
    }

}
