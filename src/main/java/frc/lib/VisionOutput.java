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
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Interpolating.Geometry.IChassisSpeeds;
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

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed, IChassisSpeeds speedSnapshot)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.standardDev = getStandardDeviation(targetsUsed, speedSnapshot);
    }

    // public VisionOutput(Pose2d estimatedPose, double timestampSeconds, Matrix<N3,N1> standardDev)  {
    //     this.estimatedPose = new Pose3d(estimatedPose);
    //     this.timestampSeconds = timestampSeconds;
    //     this.targetsUsed = null;
    //     this.standardDev = standardDev;
    // }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, PhotonTrackedTarget targetsUsed, IChassisSpeeds speedSnapshot)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = Collections.singletonList(targetsUsed);
        this.standardDev = getStandardDeviation(this.targetsUsed, speedSnapshot);
    }

    public VisionOutput(EstimatedRobotPose pose, IChassisSpeeds speedSnapshot)  {
        this(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed, speedSnapshot);
    }
    
    // public VisionOutput(PoseEstimate poseEstimate){
    //     this(new Pose3d(poseEstimate.pose), poseEstimate.timestampSeconds, getStandardDeviation(poseEstimate));
    // }

    private static Matrix<N3,N1> getStandardDeviation(PoseEstimate limelighEstimate) {
        return VecBuilder.fill(0, 0, 0); //If we are not using limelight im not going to do this one
    }

    private static Matrix<N3,N1> getStandardDeviation(List<PhotonTrackedTarget> targets, IChassisSpeeds speedSnapshot) {
        if (targets.isEmpty()) return VecBuilder.fill(0.05, 0.05, 0.05); // Set default closer to known real-world values
    
        double stddevX = 0.04
            + (0.025 * speedSnapshot.getVx() / 6) 
            + 0.06 * Math.pow((speedSnapshot.getOmega() / (Math.PI / 2)), 1.5) 
            + 0.08 * ((speedSnapshot.getVx() / 6) * (speedSnapshot.getOmega() / (Math.PI / 2)));

        double stddevY = 0.04
            + (0.025 * speedSnapshot.getVy() / 6) 
            + 0.06 * Math.pow((speedSnapshot.getOmega() / (Math.PI / 2)), 1.5) 
            + 0.08 * ((speedSnapshot.getVy() / 6) * (speedSnapshot.getOmega() / (Math.PI / 2)));

        double stddevTheta = 0.035
            + (0.025 * speedSnapshot.toMagnitude() / 6) 
            + 0.06 * Math.pow((speedSnapshot.getOmega() / (Math.PI / 2)), 1.5) 
            + 0.08 * ((speedSnapshot.toMagnitude() / 6) * (speedSnapshot.getOmega() / (Math.PI / 2)));

        if(targets.size() >= 2)
            stddevX /= 1.75;
            stddevY /= 1.75;
            stddevTheta /= 1.75;
        
        System.out.println(stddevX);
    
        return VecBuilder.fill(stddevX, stddevY, stddevTheta);
    }
    

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }
}