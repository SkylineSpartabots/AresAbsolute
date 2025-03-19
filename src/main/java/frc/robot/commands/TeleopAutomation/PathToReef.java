package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class PathToReef extends Command {
        private final CommandSwerveDrivetrain s_Swerve;

        private Supplier<ReefPoleScoringPoses> targetReefPole;
        private ReefSidePositions targetReefSide;

        private final CommandXboxController driver;

        private Command pathFindingCommand;

        public PathToReef(Supplier<ReefPoleScoringPoses> targetReefPole, CommandXboxController driver) {
                this.s_Swerve = CommandSwerveDrivetrain.getInstance();

                this.targetReefPole = targetReefPole;
                this.driver = driver;
        }

        @Override
        public void initialize() {
                if(Constants.alliance == Alliance.Blue)
                        this.targetReefSide = ReefSidePositions.values()[(int) (targetReefPole.get().ordinal() / 2)];
                else 
                        this.targetReefSide = ReefSidePositions.values()[6 + (int)((targetReefPole.get().ordinal() - 12) / 2)];

                pathFindingCommand = AutoBuilder.pathfindToPose(
                        targetReefSide.getPose(),
                        fastConstraints,
                        0.1
                );
        }

        @Override
        public void execute() {
                if(pathFindingCommand.isScheduled) {

                        if(Math.abs(driver.getLeftY()) > Constants.stickDeadband //resume path if control is let go
                        || Math.abs(driver.getLeftX()) > Constants.stickDeadband
                        || Math.abs(driver.getRightX()) > Constants.stickDeadband) {
                                pathFindingCommand.cancel(); }

                        if(pathfindingCommand.isFinished()) {
                                this.end(false); } //if the path is at its end point end the command

                } else if (Math.abs(driver.getLeftY()) < Constants.stickDeadband
                && Math.abs(driver.getLeftX()) < Constants.stickDeadband
                && Math.abs(driver.getRightX()) < Constants.stickDeadband) {
                        pathFindingCommand.schedule();
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                if(pathfindingCommand.isScheduled) { //safeguard
                        pathfindingCommand.cancel();
                }
        }
}