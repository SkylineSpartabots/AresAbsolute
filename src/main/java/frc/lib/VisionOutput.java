package frc.lib;

import java.util.Collections;
import java.util.List;

import frc.robot.LimelightHelpers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Interpolating.Geometry.ITranslation2d;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
/**
 * VisionOutput
 */
public class VisionOutput {

    /** The estimated pose */
    public final Pose3d estimatedPose;

    /** The estimated time the frame used to derive the robot pose was taken */
    public final double timestampSeconds;

    /** A list of the targets used to compute this pose */
    public final List<PhotonTrackedTarget> targetsUsed;

    public final double standardDev;

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.standardDev = getStandardDeviation(targetsUsed);
    }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, double standardDev)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = null;
        this.standardDev = getStandardDeviation(targetsUsed);
    }

    public VisionOutput(Pose2d estimatedPose, double timestampSeconds, double standardDev)  {
        this.estimatedPose = new Pose3d(estimatedPose);
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = null;
        this.standardDev = standardDev;
    }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, PhotonTrackedTarget targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = Collections.singletonList(targetsUsed);
        this.standardDev = getStandardDeviation(this.targetsUsed);
    }

    public VisionOutput(EstimatedRobotPose pose)  {
        this(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed);
    }
    
    public VisionOutput(PoseEstimate poseEstimate){
        this(new Pose3d(poseEstimate.pose), poseEstimate.timestampSeconds, getStandardDeviation(poseEstimate));
    }

    private static double getStandardDeviation(PoseEstimate limelighEstimate) {
        return -1; //If we are not using limelight im not going to do this one
    }

    private static double getStandardDeviation(List<PhotonTrackedTarget> targets) {
        int count = 0;

        for (PhotonTrackedTarget photonTrackedTarget : targets) {

            // photonTrackedTarget.area;
            // photonTrackedTarget.skew;
            // photonTrackedTarget.poseAmbiguity;
        }
        return -1;
    }

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }
}