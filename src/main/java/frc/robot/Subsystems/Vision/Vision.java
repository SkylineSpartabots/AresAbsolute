package frc.robot.Subsystems.Vision;    

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

// import org.littletonrobotics.junction.Logger;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
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

    private static List<PhotonPipelineResult> FLcameraResult;
    private static List<PhotonPipelineResult> FRcameraResult;

    private double lastProcessedTimestamp = -1;

    CommandSwerveDrivetrain s_Swerve;
    LimelightSubsystem s_Lime;
    RobotState robotState;
    
    public double floorDistance;

    private Transform3d FLcameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(-10.801), Units.inchesToMeters(-11.559), Units.inchesToMeters(-9.841)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(5),Units.degreesToRadians(0)));
    
        private Transform3d FRcameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(10.801), Units.inchesToMeters(-11.559), Units.inchesToMeters(-9.841)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(5),Units.degreesToRadians(0)));
    
        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // This is the field type that will be in PNW events

        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FLcameraToRobotTransform);
        
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
        FLcameraResult = FLCamera.getAllUnreadResults();
        FRcameraResult = FRCamera.getAllUnreadResults();
        // unreadResults.addAll(elevatorCamera.getAllUnreadResults());
    }

    public boolean validateTarget(PhotonPipelineResult camera) {
        PhotonTrackedTarget target = camera.getBestTarget();
        
        if(camera.hasTargets()
        && target.getFiducialId() >= 1
        && target.getFiducialId() <= Constants.VisionConstants.aprilTagMax
        && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1)
            return true;

        return false;
    }

    private List<MultiTagOutput> updateMultiTag(List<PhotonPipelineResult> cameraResult) {
        List<MultiTagOutput> multitags = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : cameraResult) {
            if(photonPipelineResult.getMultiTagResult().isPresent() && multitagChecks(photonPipelineResult)) {
                multitags.add(new MultiTagOutput(
                    photonPipelineResult.getMultiTagResult().get(),
                    photonPipelineResult.getTimestampSeconds(),
                    photonPipelineResult.getBestTarget()));
            }
        }

        return multitags;
    }

    private Boolean multitagChecks(PhotonPipelineResult photonPipelineResult) {

        MultiTargetPNPResult multiTagResult = photonPipelineResult.getMultiTagResult().get();
        
        System.out.println("Reproj error: " + multiTagResult.estimatedPose.bestReprojErr);
        if(multiTagResult.estimatedPose.bestReprojErr > VisionLimits.k_reprojectionLimit) {
            SmartDashboard.putString("Multitag updates", "high error");
            // Logger.recordOutput("Vision/MultiTag updates", "high error");
            return false;
        }

        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            SmartDashboard.putString("Multitag updates", "insufficient ids");
            // Logger.recordOutput("Vision/MultiTag updates", "insufficient ids");
            return false;
        } 
        System.out.println("Norm: " + multiTagResult.estimatedPose.best.getTranslation().getNorm());
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.k_normThreshold) {
            SmartDashboard.putString("Multitag updates", "norm check failed");
            // Logger.recordOutput("Vision/MultiTag updates", "norm check failed");
            return false;
        }
        System.out.println("Ambiguity: " + multiTagResult.estimatedPose.ambiguity);
        if(multiTagResult.estimatedPose.ambiguity > VisionLimits.k_ambiguityLimit) {
            SmartDashboard.putString("Multitag updates", "high ambiguity");
            // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
            return false;
        }
        for (PhotonTrackedTarget photonTrackedTarget : photonPipelineResult.getTargets()) {
            System.out.println("Area: " + multiTagResult.estimatedPose.ambiguity);
            if(photonTrackedTarget.area < VisionLimits.k_areaMinimum) {
                SmartDashboard.putString("Multitag updates", "Tag too far");
                // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
                return false;
            }

            //trasform to skew
            // if(photonTrackedTarget. < VisionLimits.k_areaMinimum) {
            //     SmartDashboard.putString("Multitag updates", "Too far");
            //     // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
            //     return false;
            // }
        }
        
        return true;
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    private void updateVision(List<PhotonPipelineResult> cameraResult, Transform3d cameraToRobotTransform) throws Exception {

        // something is wrong with this... (we still need it tho)
        // if(Math.abs(robotState.robotAngularVelocityMagnitude()[0]) > VisionLimits.k_rotationLimit) {
        //     SmartDashboard.putString("Vision accepter", "Vision failed: High rotation");
        //     return;
        // }
        
        //get data from camera
        List<MultiTagOutput> multiTagResult = updateMultiTag(cameraResult);

        if(!multiTagResult.isEmpty()) { //Use multitag if available
            for (MultiTagOutput multiTagOutput : multiTagResult) {

                //Multitag gives fieldToCamera 
                Pose3d robotPose = new Pose3d() //start at 0,0
                    .plus(multiTagOutput.getMultiTag().estimatedPose.best) //transform to camera
                        .plus(cameraToRobotTransform); //transform to robot
    
                VisionOutput newPose = new VisionOutput(robotPose, multiTagOutput.getTimestamp(),  multiTagOutput.getBestTarget());
                
                robotState.visionUpdate(newPose);  
            }
        
        } else { // if no multitags, use other tag data
            for (PhotonPipelineResult photonPipelineResult : cameraResult) {
                if(validateTarget(photonPipelineResult)) {
                    VisionOutput newPose = new VisionOutput(photonPoseEstimator.update(photonPipelineResult).get());
                    System.out.println("Used single tag");
                    robotState.visionUpdate(newPose); 
                }
            } 
        }
    }

    @Override
    public void periodic() {
        updateAprilTagResults();
        try {

        if(!FLcameraResult.isEmpty()) 
            updateVision(FLcameraResult, FLcameraToRobotTransform);

        if(!FRcameraResult.isEmpty()) 
            updateVision(FRcameraResult, FRcameraToRobotTransform);

        } catch (Exception e){}

    }
}