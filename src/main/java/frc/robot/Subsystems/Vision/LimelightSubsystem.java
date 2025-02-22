package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
//import frc.lib.vision.LimeLight; // not using the lib anymore
import frc.lib.VisionOutput;
import frc.robot.Constants;
import static frc.robot.Constants.LimelightConstants.*;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance;

    private NetworkTable nt;
    // Two ways to get data: the Limelight Helpers class (preferred) and the limeLight object which provides access to a library someone else made and i thought would be useful
//    private LimeLight limeLight; // lib
    private final String name = cameraName; // simplified for convenience

    private final CommandSwerveDrivetrain drivetrain;
    private final RobotState robotState;


    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    private LimelightSubsystem() {
//        limeLight = new LimeLight(); // init lib
        nt = NetworkTableInstance.getDefault().getTable(name);
        drivetrain = CommandSwerveDrivetrain.getInstance();
        robotState = RobotState.getInstance();

        LimelightHelpers.setCameraPose_RobotSpace(name,
                forward, right, up,
                roll, pitch, yaw);
    }

    public enum LedMode {
        pipeline(0),   //0	use the LED Mode set in the current pipeline
        forceOff(1),   //1	force off
        forceBlink(2), //2	force blink
        forceOn(3);    //3	force on 

        private static final Map<Double, LedMode> MY_MAP = new HashMap<Double, LedMode>();

        static {
            for (LedMode LedMode : values()) {
                MY_MAP.put(LedMode.getValue(), LedMode);
            }
        }

        private double value;

        LedMode(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static LedMode getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }

    }

    public enum Pipeline{
        Coral_Detector(0),
        Color_Detection(1);
        private static final Map<Integer, Pipeline> MY_MAP = new HashMap<Integer, Pipeline>();

        static {
            for (Pipeline Pipeline : values()) {
                MY_MAP.put(Pipeline.getIndex(), Pipeline);
            }
        }

        private int index;

        Pipeline(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        public static Pipeline getByIndex(int index) {
            return MY_MAP.get(index);
        }

        public String toString() {
            return name();
        }

    }

    @AutoLogOutput(key = "Limelight/Connected")
    public boolean isConnected() {
        return NetworkTableInstance.getDefault().getTable(name).containsKey("ledMode");
    }

     @AutoLogOutput(key = "Limelight/hasTarget")
     public boolean hasTarget() {
          return LimelightHelpers.getTV(name);
     }
     
    @AutoLogOutput(key = "Limelight/TargetArea")
    public double getTargetArea() {
         return LimelightHelpers.getTA(name);
    }
    @AutoLogOutput(key = "Limelight/xOffset")
    public double getXOffset() {
         return LimelightHelpers.getTX(name);
    }
    @AutoLogOutput(key = "Limelight/yOffset")
    public double getYOffset() {
         return LimelightHelpers.getTY(name);
    }
    @AutoLogOutput(key = "Limelight/MegaTagRobotPose2d")
    public Pose2d getMegaTagRobotPose2d() {return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose;}
    @AutoLogOutput(key = "Limelight/RobotPose3d")
    public Pose3d getRobotPose3d() {return  LimelightHelpers.getBotPose3d_wpiBlue(name);}

    @AutoLogOutput(key = "Limelight/LEDMode")
    public LedMode getLEDMode(){
        return LedMode.getByValue( nt.getEntry("ledMode").getInteger(-1));
    }
    @AutoLogOutput(key = "Limelight/DetectorClass")
    public String getDetectorClass(){
        return LimelightHelpers.getDetectorClass(name);
    }
    @AutoLogOutput(key = "Limelight/PipelineLatency")
    public double getPipelineLatency(){
        return LimelightHelpers.getLatency_Pipeline(name);
    }
    @AutoLogOutput(key = "Limelight/PipelineIndex")
    public int getCurrentPipelineIndex(){
        return (int) LimelightHelpers.getCurrentPipelineIndex(name);
    }

    /**
     * 
     * @return The targets 3d pose with respect to the robots coordinate system
     */
    @AutoLogOutput(key = "Limelight/TargetToRobotPose")
    public Pose3d getTargetToRobotPose(){
        return  LimelightHelpers.getTargetPose3d_RobotSpace(name);
    }
    public void setLEDMode(LedMode mode) {
        nt.getEntry("ledMode").setNumber(mode.getValue());
    }
    public void setPipeline(Pipeline pipeline){
        LimelightHelpers.setPipelineIndex(name, pipeline.getIndex());
    }


    @Override
    public void periodic() {
        //       Logger.recordOutput("Limelight/xOffset", getXOffset());
        //       Logger.recordOutput("Limelight/yOffset", getYOffset());
        //    Logger.recordOutput("Limelight/Distance", getDistance());
        //    SmartDashboard.putNumber("LimelightX", getXOffset());
        //       SmartDashboard.putNumber("LimelightY", getYOffset());
        //        SmartDashboard.putNumber("Limelight Distance", getDistance());
    }
}