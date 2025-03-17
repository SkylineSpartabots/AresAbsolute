package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

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


public class PathToReef extends Command {
        private final CommandSwerveDrivetrain s_Swerve;

        private Supplier<ReefPoleScoringPoses> targetReefPole;
        private ReefSidePositions targetReefSide;

        private final boolean opp;

        PathConstraints fastConstraints = new PathConstraints(4.5, 3.0, 3.0, 2.0); // Max speed and accel
        PIDConstants fastDrivePID = new PIDConstants(0, 0, 0);
        PIDConstants fastThetaPID = new PIDConstants(0, 0, 0);

        public PathToReef(Supplier<ReefPoleScoringPoses> targetReefPole, boolean opp) {
                this.s_Swerve = CommandSwerveDrivetrain.getInstance();

                this.opp = opp;
                this.targetReefPole = targetReefPole;
                
                addRequirements(s_Swerve);
        }

        @Override
        public void initialize() {

        if(Constants.alliance == Alliance.Blue)
                this.targetReefSide = ReefSidePositions.values()[(int) (targetReefPole.get().ordinal() / 2)];
        else 
                this.targetReefSide = ReefSidePositions.values()[6 + (int)((targetReefPole.get().ordinal() - 12) / 2)];

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                s_Swerve.getPose(),
                targetReefSide.getPose()
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                fastConstraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.35, (targetReefSide.getPose().getRotation()))
        ); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.

        path.preventFlipping = true;

        s_Swerve.followPathCommand(path, fastDrivePID, fastThetaPID).schedule();
        }

        @Override
        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
        }
}