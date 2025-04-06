package frc.robot.commands.TeleopAutomation;

import java.awt.Robot;
import java.util.Vector;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.Interpolating.Geometry.IChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.CommandFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Drives to a specified pose.
 */
public class ReefAlign extends Command {
        
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            2.9, 0.3, 0.01, new TrapezoidProfile.Constraints(Constants.MaxSpeed + 0.5, Constants.MaxAcceleration + 3.8), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            2.3, 0.8, 0, new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity, Constants.MaxAngularRate), 0.02);

    private CommandSwerveDrivetrain s_Swerve;
    private EndEffector s_EndEffector;

    private Supplier<ReefPoleScoringPoses> targetReefPole; 

    private Pose2d targetPose;
    private RobotState robotState;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 1.2; // used to scale the feedforward based on distance to target pose, this allows for a curve path to be followed when close to the target.
    private double curveScalerMaxRadius = 1, curveOffset = 1.85; //curve offset closer to 1 is end 
    public double curveRating; 
    private Boolean curveDirectionLeft; //true is left, false is right
    private double initialDistance;

    private Alliance alliance;

    public ReefAlign(Supplier<ReefPoleScoringPoses> pole) {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.robotState = RobotState.getInstance();
        this.s_EndEffector = EndEffector.getInstance();

        alliance = DriverStation.getAlliance().get();
        
        this.targetReefPole = pole;

        thetaController.setTolerance(0.3); //less than 3 degrees
        driveController.setTolerance(0.125, 0.05);

        addRequirements(s_Swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);       
    }
    
    @Override
    public void initialize() {
        Pose2d currentPose = s_Swerve.getPose();

        if(alliance == Alliance.Blue) {
                ReefSidePositions reefSide = ReefSidePositions.values()[(int) (targetReefPole.get().ordinal() / 2)];
                curveRating = reefSide.getReefSideRating(currentPose);
                targetPose = reefSide.getPose();
                curveDirectionLeft = pointDirection(currentPose, ReefConstants.reefMiddleBlue, targetPose);
        }
        else {
                ReefSidePositions reefSide = ReefSidePositions.values()[6 + (int)((targetReefPole.get().ordinal() - 12) / 2)];
                curveRating = reefSide.getReefSideRating(currentPose);
                targetPose = reefSide.getPose();
                curveDirectionLeft = pointDirection(currentPose, ReefConstants.reefMiddleRed, targetPose);
        }

        System.out.println("curve rating " + curveRating);

        initialDistance = s_Swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());

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

        if(currentDistance > 5) {
                thetaController.setP(0.5);
        } else {
                thetaController.setP(2.3);
        }

        //pause theta vel when too far
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

        // curve path based on pole
        // if(currentDistance < curveScalerMaxRadius) {
        //         double curveIntensity = MathUtil.clamp(Math.pow(Math.abs(1 - Math.abs((curveOffset * currentDistance - initialDistance)/initialDistance)), 2),
        //          0, 1); // 0 - 1 how close we are to the midpoint of the path

        //          System.out.println("curve intensity " + curveIntensity);

        //          //field centric so we dont need to adjust based on alliance
        //         double curveVectorX = curveDirectionLeft ? -driveVelocity.getX() : driveVelocity.getX();
        //         double curveVectorY = curveDirectionLeft ? driveVelocity.getY() : -driveVelocity.getY();

        //         double scale = Math.hypot(curveVectorX, curveVectorY);

        //         curveVectorX /= scale;
        //         curveVectorY /= scale;

        //         System.out.println("curve vector x" + curveVectorX);
        //         System.out.println("curve vector x" + curveVectorY);
                        
        //         driveVelocity.plus(new Translation2d(curveVectorX * curveIntensity * curveRating, curveVectorY * curveIntensity * curveRating));
        // }

        s_Swerve.applyFieldSpeeds(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity));

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
        return targetPose.equals(null) || Math.abs(driveErrorAbs) < driveController.getPositionTolerance() && Math.abs(thetaErrorAbs) < thetaController.getPositionTolerance();
    }

    // https://www.desmos.com/calculator/qghiccdvqx
    public boolean pointDirection(Pose2d currentPose, Pose2d reefMiddle, Pose2d targetPose) {
        double abX = reefMiddle.getX() - currentPose.getX();
        double abY = reefMiddle.getY() - currentPose.getY();
    
        double acX = targetPose.getX() - currentPose.getX();
        double acY = targetPose.getY() - currentPose.getY();
    
        double crossProduct = abX * acY - abY * acX;
    
        // If cross product is positive, the point is to the left
        return crossProduct > 0 ? false : true;
    }
}