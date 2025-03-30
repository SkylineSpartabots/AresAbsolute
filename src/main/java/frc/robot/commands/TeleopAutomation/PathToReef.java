package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

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
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class PathToReef extends Command {
        private final CommandSwerveDrivetrain s_Swerve;

        private Supplier<ReefPoleScoringPoses> targetReefPole;
        private ReefSidePositions targetReefSide;

        private final CommandXboxController driver;
        private DriveControlSystems controlSystem  = new DriveControlSystems();

        private Command pathFindCommand;

        private boolean forceEnd = false;

        private PathConstraints constraints = new PathConstraints(
                Constants.MaxSpeed + 1,
                Constants.MaxAcceleration + 4,
                Constants.MaxAngularVelocity + Math.PI,
                Constants.MaxAngularRate
        );

        public PathToReef(Supplier<ReefPoleScoringPoses> targetReefPole, CommandXboxController driver) {
                this.s_Swerve = CommandSwerveDrivetrain.getInstance();

                this.targetReefPole = targetReefPole;
                this.driver = driver;
        }

        @Override
        public void initialize() {

                // s_Swerve.initAutoBuilder();
                if(Constants.alliance == Alliance.Blue)
                        this.targetReefSide = ReefSidePositions.values()[(int) (targetReefPole.get().ordinal() / 2)];
                else 
                        this.targetReefSide = ReefSidePositions.values()[6 + (int)((targetReefPole.get().ordinal() - 12) / 2)];
                
                s_Swerve.resetOdo(s_Swerve.getPose());

                if (pathFindCommand != null && pathFindCommand.isScheduled()) {
                        pathFindCommand.cancel();
                }

                System.out.println("target pose" + targetReefSide.getPose().toString());

                pathFindCommand = AutoBuilder.pathfindToPose(
                        targetReefSide.getPose(),
                        constraints,
                        0
                );

                pathFindCommand.schedule();
        }

        @Override
        public void execute() {
                // System.out.println(s_Swerve.getCurrentCommand().getName());

                // System.out.println("comming still going");
                // if(pathFindCommand.isScheduled()) {

                //         Pose2d pose = s_Swerve.getPose();
                //         if(Math.abs(pose.getX() - targetReefSide.getPose().getX()) < 0.01 
                //         && Math.abs(pose.getY() - targetReefSide.getPose().getY()) < 0.01 
                //         && Math.abs(pose.getRotation().minus(targetReefSide.getPose().getRotation()).getDegrees()) < 6)  {
                //                 forceEnd = true;
                //         }
                         
                //         if(Math.abs(driver.getLeftY()) > Constants.stickDeadband // cancel path if driver wants to move
                //                 || Math.abs(driver.getLeftX()) > Constants.stickDeadband
                //                 || Math.abs(driver.getRightX()) > Constants.stickDeadband) {
                //                 pathFindCommand.cancel();
                //         }

                // } else if(Math.abs(driver.getLeftY()) < Constants.stickDeadband
                //  && Math.abs(driver.getLeftX()) < Constants.stickDeadband
                //  && Math.abs(driver.getRightX()) < Constants.stickDeadband) {
                //         pathFindCommand.schedule();
                //  }

        }

        @Override
        public boolean isFinished() {
                return pathFindCommand.isFinished() || forceEnd;
        }

        @Override
        public void end(boolean interrupted) {
                if(pathFindCommand.isScheduled()) { //safeguard
                        pathFindCommand.cancel();
                }
        }
}