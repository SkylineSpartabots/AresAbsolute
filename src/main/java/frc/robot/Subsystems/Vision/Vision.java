package frc.robot.Subsystems.Vision;    

import java.util.ArrayList;
import java.util.List;

// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.MultiTagOutput;
import frc.lib.VisionOutput;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.VisionLimits;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera FLCamera;
    private static PhotonCamera FRCamera;
    private static PhotonCamera elevatorCamera;

    private static List<PhotonPipelineResult> cameraResults;

    private double lastProcessedTimestamp = -1;

    CommandSwerveDrivetrain s_Swerve;
    LimelightSubsystem s_Lime;
    RobotState robotState;
    
    private Transform3d cameraToRobotTransform = new Transform3d( // put in constants
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(0)));

        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // This is the field type that will be in PNW events

        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobotTransform);
        
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
    
    public Vision() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        robotState = RobotState.getInstance();

        FLCamera = new PhotonCamera(Constants.VisionConstants.FLCamera);
        FRCamera = new PhotonCamera(Constants.VisionConstants.FRCamera);
        elevatorCamera = new PhotonCamera(Constants.VisionConstants.elevatorCamera);

        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        List<PhotonPipelineResult> unreadResults = FLCamera.getAllUnreadResults();
        unreadResults.addAll(FRCamera.getAllUnreadResults());
        unreadResults.addAll(elevatorCamera.getAllUnreadResults());
        
        if (!unreadResults.isEmpty()){
            cameraResults = unreadResults; // assuming first index is the latest result
        }
        // Zero yaw is robot facing red alliance wall - our code should be doing this.
    }

    public boolean validateTarget(PhotonPipelineResult result) {
        PhotonTrackedTarget target = result.getBestTarget();
        

        return result.hasTargets()
                && target.getFiducialId() >= 1
                && target.getFiducialId() <= Constants.VisionConstants.aprilTagMax
                && target.getPoseAmbiguity() < VisionLimits.ambiguityLimit && target.getPoseAmbiguity() > -1;
    }

    private List<MultiTagOutput> updateMultiTag() {
        List<MultiTagOutput> multitags = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : cameraResults) {
            if(photonPipelineResult.getMultiTagResult().isPresent() && multitagChecks(photonPipelineResult.getMultiTagResult().get())) {
                multitags.add(new MultiTagOutput(photonPipelineResult.getMultiTagResult().get(), photonPipelineResult.getTimestampSeconds(), photonPipelineResult.getBestTarget()));
            }
        }

        return multitags;
    }

    private Boolean multitagChecks(MultiTargetPNPResult multiTagResult) {

        if(multiTagResult.estimatedPose.bestReprojErr > VisionLimits.reprojectionLimit) {
             Logger.recordOutput("Vision/MultiTag updates", "high error");
            return false;
        }
        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
             Logger.recordOutput("Vision/MultiTag updates", "insufficient ids");
            return false;
        } 
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.normThreshold) {
             Logger.recordOutput("Vision/MultiTag updates", "norm check failed");
            return false;
        } 
        if(multiTagResult.estimatedPose.ambiguity > VisionLimits.ambiguityLimit) {
             Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
            return false;
        }

        //in the future we would have a set of tags we would only want to mega tag
        // for (var fiducialID : multiTagResult.fiducialIDsUsed) {
        //     if (fiducialID =! idk) {
        //     }
        // }

        return true;
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    private void updateVision() throws Exception {

        // something is wrong with this... (we still need it tho)
        // if(Math.abs(robotState.robotAngularVelocityMagnitude()[0]) > VisionLimits.k_rotationLimit) {
        //     SmartDashboard.putString("Vision accepter", "Vision failed: High rotation");
        //     return;
        // }
        
        //get data from camera
        List<MultiTagOutput> multiTagResult = updateMultiTag();

        if(!multiTagResult.isEmpty()) { //Use multitag if available
            for (MultiTagOutput multiTagOutput : multiTagResult) {
                Pose3d tagPose = aprilTagFieldLayout.getTagPose(multiTagOutput.getBestTarget().getFiducialId()).get();

                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(multiTagOutput.estimatedPose.best, tagPose, cameraToRobotTransform);
    
                VisionOutput newPose = new VisionOutput(robotPose, multiTagOutput.getTimestamp(),  multiTagOutput.getBestTarget());
                
                System.out.println(newPose.toString());
    
                robotState.visionUpdate(newPose);    
            }
        } else { // if no multitags, use other tag data
            for (PhotonPipelineResult photonPipelineResult : cameraResults) {
                if(validateTarget(photonPipelineResult)) {
                    VisionOutput newPose = new VisionOutput(photonPoseEstimator.update(photonPipelineResult).get());
                    robotState.visionUpdate(newPose); 
                }
            } 
        }
    }

    @Override
    public void periodic() {
        // updateAprilTagResults();
        // if(!cameraResult.isEmpty()) {
        //     try {
        //         updateVision();
        //     } catch (Exception e){}
        // } 
    }
}