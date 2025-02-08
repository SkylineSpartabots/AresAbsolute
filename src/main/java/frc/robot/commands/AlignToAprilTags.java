package frc.robot.commands;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;

public class AlignToAprilTags extends Command{
    private Vision vision;
    private CommandSwerveDrivetrain s_swerve;
    // private int apriltagID;
    private RobotState robotState;
    private PIDController pid = new PIDController(0, 0, 0);
    private PhotonTrackedTarget target;

    public AlignToAprilTags() {
        // this.apriltagID = apriltagID;
        s_swerve = CommandSwerveDrivetrain.getInstance();
        vision = Vision.getInstance();
        robotState = RobotState.getInstance();
    }

    public PhotonTrackedTarget getTarget() {
        double biggestTargetArea = 0.0;
        double targetArea = 0.0;
        PhotonTrackedTarget goodTarget = null;
        var results = vision.getLatestAprilTagResult();
        if(results != null) {
            if(results.hasTargets()) {
                for(var target : results.getTargets()){
                    targetArea = target.getArea();
                    if(targetArea > biggestTargetArea) {
                        biggestTargetArea = targetArea;
                        goodTarget = target;
                    }
                }
            }
        } else {
            System.out.println("No April Tag Found");
        }

        return goodTarget;
    }


    public double[] getTargetValues() {
        double[] values = new double[3];

        double targetYaw = 0.0;
        double targetX = 0.0;
        double targetY = 0.0;
        if(target == null) {
            target = getTarget();
        }

        targetYaw = target.getYaw();
        targetX = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.centerCameraHeight, 
            Constants.VisionConstants.aprilTagHeight, 
            Constants.VisionConstants.centerCameraPitch, 
            Units.degreesToRadians(target.getPitch()));
        targetY = targetX * Math.sin(Units.degreesToRadians(targetYaw));

        System.out.println("April Tag ID: " + target.getFiducialId());
        System.out.println("Target Yaw: " + targetYaw);
        System.out.println("Target Range: " + targetX);
        System.out.println("Target Strafe: " + targetY);

        values[0] = targetYaw;
        values[1] = targetX;
        values[2] = targetY;

        return values;
    }

    @Override
    public void initialize() {
        pid.setPID(Constants.robotPIDs.AprilTagAlignmentPID.kP, 
            0, 
            Constants.robotPIDs.AprilTagAlignmentPID.kD);
    }

    @Override
    public void execute() {
        double[] values = getTargetValues();

        double turn = pid.calculate(s_swerve.getHeading(), values[0]);
        double forward = pid.calculate(1.0, values[1]);
        double strafe = pid.calculate(0.0, values[2]);

        Pose2d currentPose = robotState.getCurrentPose2d();

        s_swerve.applyFieldSpeeds((ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, turn, currentPose.getRotation())));
        
        SmartDashboard.putNumber("Yaw Error: ", values[0] - s_swerve.getHeading());
        SmartDashboard.putNumber("Range Error: ", 1.0 - values[1]);
        SmartDashboard.putNumber("Strafe Error: ", values[2]);
    }

    @Override
    public boolean isFinished() {
        double[] values = getTargetValues();
        return (((values[0] - s_swerve.getHeading()) == 0) && ((1.0 - values[1]) == 0) && (values[2] == 0));
    }

    @Override
    public void end(boolean interrupted) {
        s_swerve.applyFieldSpeeds(new ChassisSpeeds());
    }
}
