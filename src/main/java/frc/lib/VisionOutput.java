package frc.lib;

import java.util.Collections;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.lib.LimelightHelpers.RawFiducial;
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

    public final Matrix<N3,N1> standardDev;

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.standardDev = getStandardDeviation(targetsUsed);
    }

    // public VisionOutput(Pose2d estimatedPose, double timestampSeconds, Matrix<N3,N1> standardDev)  {
    //     this.estimatedPose = new Pose3d(estimatedPose);
    //     this.timestampSeconds = timestampSeconds;
    //     this.targetsUsed = null;
    //     this.standardDev = standardDev;
    // }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, PhotonTrackedTarget targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = Collections.singletonList(targetsUsed);
        this.standardDev = getStandardDeviation(this.targetsUsed);
    }

    public VisionOutput(EstimatedRobotPose pose)  {
        this(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed);
    }
    
    // public VisionOutput(PoseEstimate poseEstimate){
    //     this(new Pose3d(poseEstimate.pose), poseEstimate.timestampSeconds, getStandardDeviation(poseEstimate));
    // }

    private static Matrix<N3,N1> getStandardDeviation(PoseEstimate limelighEstimate) {
        return VecBuilder.fill(0, 0, 0); //If we are not using limelight im not going to do this one
    }

    private static Matrix<N3,N1> getStandardDeviation(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return VecBuilder.fill(0, 0, 0); // Set default closer to known real-world values
    
        double totalError = 0;
        int count = 0;
    
        for (PhotonTrackedTarget target : targets) {
            double ambiguityFactor = target.poseAmbiguity; // 0 - 1
            double areaFactor = target.area; // Reduced impact of area 0 - 1
            double skewFactor = target.skew; // Lower skew contribution
    
            double error = ambiguityFactor + areaFactor + skewFactor;
            totalError += error;
            count++;
        }
    
        // double baseError = 0.002; // Baseline error for robustness
        // return count > 0 ? Math.max(baseError, totalError / count) : baseError;
        return VecBuilder.fill(0, 0, 0);
    }
    

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }
}