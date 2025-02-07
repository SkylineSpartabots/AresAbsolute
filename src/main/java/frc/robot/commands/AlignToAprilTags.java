package frc.robot.commands;
import org.photonvision.PhotonUtils;

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

    public AlignToAprilTags() {
        // this.apriltagID = apriltagID;
        s_swerve = CommandSwerveDrivetrain.getInstance();
        vision = Vision.getInstance();
        robotState = RobotState.getInstance();
    }

    public double[] getTargetValues() {
        double[] values = new double[3];

        double targetYaw = 0.0;
        double targetRange = 0.0;
        double targetStrafe = 0.0;
        var result = vision.getLatestAprilTagResult();
        if(result != null) {
            if(result.hasTargets()) {
                for(var target : result.getTargets()){
                    targetYaw = target.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.centerCameraHeight, 
                        Constants.VisionConstants.aprilTagHeight, 
                        Constants.VisionConstants.centerCameraPitch, 
                        Units.degreesToRadians(target.getPitch()));
                    targetStrafe = targetRange * Math.sin(Units.degreesToRadians(targetYaw));
                }
            }
        } else {
            System.out.println("No April Tag Found");
        }

        values[0] = targetYaw;
        values[1] = targetRange;
        values[2] = targetStrafe;

        return values;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        double prevYawError = 0.0;
        double prevRangeError = 0.0;
        double prevStrafeError = 0.0;
        double deltaTime = 0.02;

        double[] values = getTargetValues();
        double yawError = values[0] - s_swerve.getHeading();
        if(yawError > 180) {
            yawError -= 360;
        } else if(yawError < 180) {
            yawError += 360;
        }

        double rangeError = 1.0 - values[1];
        double strafeError = values[2];

        double dYawError = (yawError - prevYawError) / deltaTime;
        double dRangeError = (rangeError - prevRangeError) / deltaTime;
        double dStrafeError = (strafeError - prevStrafeError) / deltaTime;

        pid.setPID(Constants.robotPIDs.AprilTagAlignmentPID.kP, 
            0, 
            Constants.robotPIDs.AprilTagAlignmentPID.kD);

        double turn = (yawError * Constants.robotPIDs.AprilTagAlignmentPID.kP * Constants.MaxAngularRate) + 
                    (dYawError * Constants.robotPIDs.AprilTagAlignmentPID.kD * Constants.MaxAngularRate);
        
        double forward = (rangeError * Constants.robotPIDs.AprilTagAlignmentPID.kP * Constants.MaxSpeed) + 
                    (dRangeError * Constants.robotPIDs.AprilTagAlignmentPID.kD * Constants.MaxSpeed);

        double strafe = (strafeError * Constants.robotPIDs.AprilTagAlignmentPID.kP * Constants.MaxSpeed) + 
                    (dStrafeError * Constants.robotPIDs.AprilTagAlignmentPID.kD * Constants.MaxSpeed);

        Pose2d currentPose = robotState.getCurrentPose2d();

        s_swerve.applyFieldSpeeds((ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, turn, currentPose.getRotation())));

        prevYawError = yawError;
        prevRangeError = rangeError;
        prevStrafeError = strafeError;
        
        SmartDashboard.putNumber("Yaw Error: ", values[0] - s_swerve.getHeading());
        SmartDashboard.putNumber("Range Error: ", 1.0 - values[1]);
    }

    @Override
    public boolean isFinished() {
        double[] values = getTargetValues();
        return (((values[0] - s_swerve.getHeading()) == 0) && ((1.0 - values[1]) == 0));
    }

    @Override
    public void end(boolean interrupted) {
        s_swerve.applyFieldSpeeds(new ChassisSpeeds());
    }
}
