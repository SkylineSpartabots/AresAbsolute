package frc.robot.commands.SwerveCommands;

import java.awt.Robot;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.Interpolating.Geometry.IChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Elevator.SetElevator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Drives to a specified pose.
 */
public class AutomatedCoralAction extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            3.75, 0.09, 0.006, new TrapezoidProfile.Constraints(Constants.MaxSpeed, Constants.MaxAcceleration), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3, 1.2, 0, new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity, Constants.MaxAngularRate), 0.02);

    private CommandSwerveDrivetrain s_Swerve;
    private EndEffector s_EndEffector;

    private Supplier<ElevatorState> elevatorLevel;
    private Supplier<ReefPoleScoringPoses> targetReefPole; 

    private Double elevatorGoalPos = Double.POSITIVE_INFINITY;

    private Alliance alliance;

    private Pose2d targetPose;
    private RobotState robotState;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 1.2, elevatorDistanceThreshold = 1, dealgeaDistanceThreshold = 0.75;



    public AutomatedCoralAction(Supplier<ElevatorState> elevatorLevel, Supplier<ReefPoleScoringPoses> pole) {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.robotState = RobotState.getInstance();
        this.s_EndEffector = EndEffector.getInstance();
        
        this.targetReefPole = pole;
        this.elevatorLevel = elevatorLevel;

        alliance = DriverStation.getAlliance().get();

        addRequirements(s_Swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);       
    }

    @Override
    public void initialize() {
        elevatorGoalPos = elevatorLevel.get().getEncoderPosition();
        targetPose = targetReefPole.get().getPose();

        Pose2d currentPose = s_Swerve.getPose();
        IChassisSpeeds speeds = robotState.getLatestFilteredVelocity();
        driveController.reset(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(speeds.getVx(),speeds.getVy())
                                .rotateBy(
                                        targetPose
                                                .getTranslation()
                                                .minus(s_Swerve.getPose().getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX())); // Distance between current and target pose

        thetaController.reset(s_Swerve.getHeading(),
                robotState.getLatestFilteredVelocity().getOmega());
        
        thetaController.setTolerance(0.05235); //3 degrees
                
        lastSetpointTranslation = s_Swerve.getPose().getTranslation();

    }

    @Override
    public void execute() {

        Pose2d currentPose = s_Swerve.getPose();

        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation()); //error between poses

        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);

        driveErrorAbs = currentDistance;
        
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
            lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        new Transform2d(new Translation2d(driveController.getSetpoint().position, 0.0), new Rotation2d()))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity = new Pose2d(new Translation2d(), currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
                .getTranslation();
                s_Swerve.applyFieldSpeeds(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity));

        // other actions
        if(!elevatorGoalPos.isInfinite() && driveErrorAbs < elevatorDistanceThreshold && !s_EndEffector.getBeamResult()) {
                new SetElevator(elevatorGoalPos).schedule();
                elevatorGoalPos = Double.POSITIVE_INFINITY;
        }

        //prints
        // System.out.println("Theta error: " + thetaErrorAbs);
        // System.out.println("drive error: " + driveErrorAbs);
        // System.out.println("Position Drivetrain error: " + driveController.getPositionError());
        // System.out.println("Drivetrain error: " + driveController.getPositionError());
        // System.out.println("Position Theta error: " + thetaController.getPositionError());
        // System.out.println("Drive velocity: " + driveVelocityScalar);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.applyFieldSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return targetPose.equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }
}