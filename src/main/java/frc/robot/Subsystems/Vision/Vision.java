package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.MultiTagOutput;
import frc.lib.VisionOutput;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.CameraNames;
import frc.robot.Constants.VisionConstants.VisionLimits;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.VisionConstants.CameraTransforms.*;

public class Vision extends SubsystemBase {
    private static Vision instance;


    private static PhotonCamera FrontLeftCamera;
    private static PhotonCamera FrontRightCamera;
    private static PhotonCamera FrontRightAngledCamera;
    private static PhotonCamera BackLeftCamera;
    private static PhotonCamera BackRightCamera;
    private static PhotonCamera BackCenterCamera;

    private static List<PhotonPipelineResult> FLcameraResult;
    private static List<PhotonPipelineResult> FRcameraResult;
    private static List<PhotonPipelineResult> FRAcameraResult;
    private static List<PhotonPipelineResult> BLcameraResult;
    private static List<PhotonPipelineResult> BRcameraResult;
    private static List<PhotonPipelineResult> BCcameraResult;

    PhotonPoseEstimator FLphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FLcameraToRobot);
    PhotonPoseEstimator FRphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRcameraToRobot);
    PhotonPoseEstimator FRAphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FCcameraToRobot);
    PhotonPoseEstimator BLphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BLcameraToRobot);
    PhotonPoseEstimator BRphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BRcameraToRobot);
    PhotonPoseEstimator BCphotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BCcameraToRobot);

    CommandSwerveDrivetrain s_Swerve;
    RobotState robotState;

    private boolean frontCamerasBool = false;

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // This is the field type that will be in PNW events

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        robotState = RobotState.getInstance();

        FrontLeftCamera = new PhotonCamera(CameraNames.FrontLeft);
        FrontRightCamera = new PhotonCamera(CameraNames.FrontRight);
        FrontRightAngledCamera = new PhotonCamera(CameraNames.FrontRightCenter);
        BackLeftCamera = new PhotonCamera(CameraNames.BackLeft);
        BackRightCamera = new PhotonCamera(CameraNames.BackRight);
        BackCenterCamera = new PhotonCamera(CameraNames.BackCenter);

        FLphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        FRphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        FRAphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        BLphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        BRphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        BCphotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

//        updateAprilTagResults();
    }

    // we refresh in updateVision now - but i will leave this method in case someone wants it
    public void updateAprilTagResults() {
        FLcameraResult = FrontLeftCamera.getAllUnreadResults();
        FRcameraResult = FrontRightCamera.getAllUnreadResults();
        FRAcameraResult = FrontRightAngledCamera.getAllUnreadResults();
        BLcameraResult = BackLeftCamera.getAllUnreadResults();
        BRcameraResult = BackRightCamera.getAllUnreadResults();
        BCcameraResult = BackCenterCamera.getAllUnreadResults();
    }

    public boolean validateTarget(PhotonPipelineResult camera) {
        if (!camera.hasTargets())
            return false;

        PhotonTrackedTarget target = camera.getBestTarget();

        if (target.area > 0.06
                && target.getFiducialId() >= 1
                && target.getFiducialId() <= Constants.VisionConstants.aprilTagMax
                && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1)
            return true;

        return false;
    }

    private List<MultiTagOutput> updateMultiTag(List<PhotonPipelineResult> cameraResult) {
        List<MultiTagOutput> multitags = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : cameraResult) {
            if (photonPipelineResult.getMultiTagResult().isPresent() && multitagChecks(photonPipelineResult)) {
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

        if (multiTagResult.estimatedPose.bestReprojErr > VisionLimits.k_reprojectionLimit) {
            // SmartDashboard.putString("Multitag updates", "high error");
            // Logger.recordOutput("Vision/MultiTag updates", "high error");
            return false;
        }

        if (multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            // SmartDashboard.putString("Multitag updates", "insufficient ids");
            // Logger.recordOutput("Vision/MultiTag updates", "insufficient ids");
            return false;
        }
        if (multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.k_normThreshold) {
            // SmartDashboard.putString("Multitag updates", "norm check failed");
            // Logger.recordOutput("Vision/MultiTag updates", "norm check failed");
            return false;
        }
        if (multiTagResult.estimatedPose.ambiguity > VisionLimits.k_ambiguityLimit) {
            // SmartDashboard.putString("Multitag updates", "high ambiguity");
            // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
            return false;
        }

        // for (PhotonTrackedTarget photonTrackedTarget : photonPipelineResult.getTargets()) {
        //     if(photonTrackedTarget.area < VisionLimits.k_areaMinimum) {
        //         SmartDashboard.putString("Multitag updates", "Tag too far");
        //         // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
        //         return false;
        //     }

        //     //trasform to skew
        //     // if(photonTrackedTarget. < VisionLimits.k_areaMinimum) {
        //     //     SmartDashboard.putString("Multitag updates", "Too far");
        //     //     // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
        //     //     return false;
        //     // }
        // }

        return true;
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    private void updateVision(PhotonCamera camera, Transform3d cameraToRobot , StandardDevs stddev) throws Exception {

        List<PhotonPipelineResult> cameraResult = camera.getAllUnreadResults();

        if (cameraResult.isEmpty()) {
            return;
        }

        // something is wrong with this... (we still need it tho)
        // if(Math.abs(robotState.robotAngularVelocityMagnitude()[0]) > VisionLimits.k_rotationLimit) {
        //     SmartDashboard.putString("Vision accepter", "Vision failed: High rotation");
        //     return;
        // }

        // if(Math.abs(robotState.robotVelocityVector()) > VisionLimits.k_velocityLimit) {
        //     SmartDashboard.putString("Vision accepter", "Vision failed: High speed");
        //     return;
        // }

        //get data from camera
        List<MultiTagOutput> multiTagResult = updateMultiTag(cameraResult);

        if (!multiTagResult.isEmpty()) { //Use multitag if available
            for (MultiTagOutput multiTagOutput : multiTagResult) {

                //Multitag gives fieldToCamera 
                Pose3d robotPose = new Pose3d() //start at 0,0
                        .plus(multiTagOutput.getMultiTag().estimatedPose.best) //transform to camera
                        .plus(cameraToRobot); //transform to robot

                VisionOutput newPose = new VisionOutput(robotPose,
                        multiTagOutput.getTimestamp(),
                        multiTagOutput.getBestTarget(),
                        robotState.getOdomRobotVelocity(Utils.fpgaToCurrentTime(multiTagOutput.getTimestamp())), true);

                s_Swerve.addVisionMeasurement(newPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(newPose.timestampSeconds), stddev.getStandardDev(true));
            }
        
        } else {
            // if no multitags, use other tag data
            for (PhotonPipelineResult photonPipelineResult : cameraResult) {
                if (validateTarget(photonPipelineResult)) {
                    VisionOutput newPose = new VisionOutput(
                            PhotonUtils.estimateFieldToRobotAprilTag(
                                    photonPipelineResult.getBestTarget().bestCameraToTarget,
                                    aprilTagFieldLayout.getTagPose(photonPipelineResult.getBestTarget().fiducialId).get(),
                                    (cameraToRobot)),
                            photonPipelineResult.getTimestampSeconds(),
                            photonPipelineResult.getBestTarget(),
                            robotState.getOdomRobotVelocity(Utils.fpgaToCurrentTime(photonPipelineResult.getTimestampSeconds())), false);

                    // System.out.println(camera.getName() + " pose: " + newPose.estimatedPose.toString());

                    s_Swerve.addVisionMeasurement(newPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(newPose.timestampSeconds), stddev.getStandardDev(false));
                }

            }

        }
    }

    

    public void useFrontCameras() {
        frontCamerasBool = !frontCamerasBool;
    }

    @Override
    public void periodic() {
        try {

            if(frontCamerasBool) { //only use front cameras
                updateVision(FrontLeftCamera, FLcameraToRobot, StandardDevs.ALIGNING_FRONT);
                updateVision(FrontRightCamera, FRcameraToRobot, StandardDevs.ALIGNING_FRONT);
                updateVision(FrontRightAngledCamera, FCcameraToRobot, StandardDevs.ALIGNING_FRONTMIDDLE);
            } else {
                updateVision(FrontLeftCamera, FLcameraToRobot, StandardDevs.DEFAULT_FRONT);
                updateVision(FrontRightCamera, FRcameraToRobot, StandardDevs.DEFAULT_FRONT);
                updateVision(FrontRightAngledCamera, FCcameraToRobot, StandardDevs.DEFAULT_FRONTMIDDLE);
                updateVision(BackLeftCamera, BLcameraToRobot, StandardDevs.DEFAULT_BACK);
                updateVision(BackRightCamera, BRcameraToRobot, StandardDevs.DEFAULT_BACK);
                updateVision(BackCenterCamera, BCcameraToRobot, StandardDevs.DEFAULT_BACK);
            }

        } catch (Exception e) {
        }
    }

    private enum StandardDevs { // # # # mulittag, # # # single tag
        DEFAULT_FRONT(VecBuilder.fill(0.013, 0.013, 0.045, //good
                                     0.0254, 0.0254, 0.08)), //good

        DEFAULT_FRONTMIDDLE(VecBuilder.fill(0.013, 0.013, 0.045, //good
                                        0.0254, 0.0254, 0.08)), //good

        DEFAULT_BACK(VecBuilder.fill(0.0165, 0.0165, 0.0165, //good
                                     0.06, 0.06, 0.1)), //good

        ALIGNING_FRONT(VecBuilder.fill(0.01, 0.01, 0.01, //good
                                        0.02, 0.02, 0.06)), //good

        ALIGNING_FRONTMIDDLE(VecBuilder.fill(0.008, 0.008, 0.04, //good
                                        0.019, 0.019, 0.05)); //good


        private Vector<N6> StandardDev;

        private StandardDevs(Vector<N6> StandardDev) {
            this.StandardDev = StandardDev;
        }

        public Vector<N3> getStandardDev(boolean mulittag) {
            Vector<N6> vec = this.StandardDev;
            return mulittag ? VecBuilder.fill(vec.get(0),vec.get(1),vec.get(2)) : VecBuilder.fill(vec.get(3),vec.get(4),vec.get(5));
        }
    }

}