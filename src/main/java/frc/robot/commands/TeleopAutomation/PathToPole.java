package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.commands.Elevator.SetElevator;

import java.util.List;
import java.util.function.Supplier;

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


public class PathToPole extends Command {
        private final CommandSwerveDrivetrain s_Swerve;

        private PathPlannerTrajectory trajectory;

        private Supplier<ReefPoleScoringPoses> targetReefPole;
        private Supplier<ElevatorState> targetElevatorLevel;

        PathConstraints preciseConstraints = new PathConstraints(1.5, 1.0, 2.0, 1.0); // Lower speed, high control
        PIDConstants preciseDrivePID = new PIDConstants(0, 0, 0);
        PIDConstants preciseThetaPID = new PIDConstants(0, 0, 0);

        public PathToPole(Supplier<ReefPoleScoringPoses> targetReefPole, Supplier<ElevatorState> targetElevatorLevel) {
                this.s_Swerve = CommandSwerveDrivetrain.getInstance();

                this.targetReefPole = targetReefPole;
                this.targetElevatorLevel = targetElevatorLevel;
                
                addRequirements(s_Swerve);
        }

        @Override
        public void initialize() {

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                s_Swerve.getPose(),
                targetReefPole.get().getPose()
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                preciseConstraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0, (targetReefPole.get().getPose().getRotation()))
        ); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.

        path.preventFlipping = true;

        //put elevator up
        new SetElevator(targetElevatorLevel).schedule();
        s_Swerve.followPathCommand(path, preciseDrivePID, preciseThetaPID).schedule();
        }

        @Override
        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
        }
}