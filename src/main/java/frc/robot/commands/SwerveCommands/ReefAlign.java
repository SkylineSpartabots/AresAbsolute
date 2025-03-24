package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Interpolating.Geometry.IChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Drives to a specified pose.
 */
public class ReefAlign extends Command {
        
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            4, 0.12, 0.05, new TrapezoidProfile.Constraints(Constants.MaxSpeed + 2, Constants.MaxAcceleration + 1), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3.254, 2, 0, new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity + Math.PI, Constants.MaxAngularRate), 0.02);

    private CommandSwerveDrivetrain s_Swerve;

    private ReefSidePositions targetReefSide; 

    private Supplier<ReefPoleScoringPoses> pole;
    private Pose2d targetPose;
    private RobotState robotState;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 1.2, elevatorDistanceThreshold = 1, dealgeaDistanceThreshold = 0.75;

//     public ReefAlign(ReefSidePositions side) {
//         this.s_Swerve = CommandSwerveDrivetrain.getInstance();
//         this.robotState = RobotState.getInstance();
        
//         this.targetReefSide = side;

//         thetaController.setTolerance(0.1047); //6 degrees
//         driveController.setTolerance(0.2);

//         addRequirements(s_Swerve);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);       
//     }

    public ReefAlign(Supplier<ReefPoleScoringPoses> pole) {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.robotState = RobotState.getInstance();
        System.out.println("my o dinal: " + (int) pole.get().ordinal() / 2);

        this.pole = pole;

        thetaController.setTolerance(0.1); //less than 6 degrees
        driveController.setTolerance(0.2);

        addRequirements(s_Swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);       
    }

    @Override
    public void initialize() {
        if(Constants.alliance == Alliance.Blue)
                this.targetReefSide = ReefSidePositions.values()[(int) (pole.get().ordinal() / 2)];
        else 
                this.targetReefSide = ReefSidePositions.values()[6 + (int)((pole.get().ordinal() - 12) / 2)];

        targetPose = targetReefSide.getPose();

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

        //prints
        // System.out.println("Theta error: " + thetaErrorAbs);()
        // System.out.println("theta error: " + thetaController.getPositionError() + (Math.abs(thetaController.getPositionError()) < 0.1));
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
        return targetPose.equals(null) || Math.abs(driveErrorAbs) < driveController.getPositionTolerance() && Math.abs(thetaErrorAbs) < thetaController.getPositionTolerance();
    }
}