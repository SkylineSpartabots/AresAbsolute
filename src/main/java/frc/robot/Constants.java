// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.AprilTags;
import frc.robot.RobotState.RobotState;

import java.util.Arrays;
import java.util.Comparator;

import org.opencv.core.Point;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static double MaxSpeed = 5.5; //can be lowered during testing
    public static double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final double dt = 0.02; // 3/4 of a rotation per second max angular velocity
    
    public static final int timeOutMs = 10;

    public static final double elevatorMaxVelocity = 5;
    public static final double elevatorMaxAcceleration = 1;

    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;
    
    public static final class OuttakePhysicalConstants{
        public static final double outtakeRollerRadius = 0;
        public static final double outtakeOffsetMillimeters = 0; //distance between center of robot and PVC center of mass after exiting outtake in mm
    }

    


    public static final double stickDeadband = 0.05;
    public static final double triggerDeadzone = 0.2;


    public static final class CurrentLimits{
        public static final int outtakeContinuousCurrentLimit = 35;
        public static final int outtakePeakCurrentLimit = 65;
        
        public static final int intakeContinuousCurrentLimit = 40;
        public static final int intakePeakCurrentLimit = 70;

        public static final int elevatorContinuousCurrentLimit = 50;
        public static final int elevatorPeakCurrentLimit = 80;
    }
    

    public static Mode deployMode = Mode.REAL;

    public static final double outtakeAngle = 35;

    public static final class TrajectoryConstants {
        public static final double maxAcceleration = 2.0;
        public static final double maxVelocity = 6.0;

        public static final double poseToleranceX = 0.02;
        public static final double poseToleranceY = 0.02;
        public static final double poseToleranceTheta = Math.PI / 30; // 6 degrees
    }

    public static final double elevatorContinuousCurrentLimit = 50;
    public static final double elevatorPeakCurrentLimit = 80;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class VisionConstants {
        public static final String cameraName = "camera";
        public static final int aprilTagMax = 16;
        public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet | unnecessary, we have photonvision's field layout import
        public static final double cameraRollOffset = Units.degreesToRadians(0);
        public static final double cameraPitchOffset = Units.degreesToRadians(0);
        public static final double cameraYawOffset = Units.degreesToRadians(0);
        public static final double backRightCameraHeight = Units.inchesToMeters(9.1);
        public static final double backRightCameraPitch = Units.degreesToRadians(30);

        public static final double centerCameraHeight = Units.inchesToMeters(10.15);
        public static final double centerCameraPitch = Units.degreesToRadians(15);

        public static final class VisionLimits {
        public static final int k_rotationLimitDPS = 175;
        public static final double k_reprojectionLimit = 0.1;
        public static final double k_normThreshold = 0.1;
        public static final double k_ambiguityLimit = 0.9;
        }

        public static final class AprilTags {
            //fill with april tags
        }
    }


    // hardware ports for all hardware components on the robot
    // these include CAN IDs, pneumatic hub ports, etc.

    public static final class robotPIDs {
        //fill with our pids

        public static final class HeadingControlPID {
            public static final double highP = 12;
            public static final double highI = 0;
            public static final double highD = 4;

            public static final double lowP = 7;
            public static final double lowI = 0;
            public static final double lowD = 1.5;
        }
    }

    public static final class HardwarePorts {
        // motor id
        public static final int topOuttake = 21;
        public static final int botOuttake = 22;
        public static final int intakeRollerID = 11;
        public static final int intakePivotID = 12;

        public static final int elevatorLeaderId = 31;
        public static final int elevatorFollowerId = 32;
    }

    //change for next game
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2022RapidReact); //WHERE IS NEW FIELD?

    public static final class FieldConstants {

        public static final Pose2d Procceser = new Pose2d(0,0, new Rotation2d(0));

        public static final class Cage { // idk what our cage mech will be but might not need this
            public static final Pose2d RedCage1 = new Pose2d(0,0, new Rotation2d(0));
            public static final Pose2d RedCage2 = new Pose2d(0,0, new Rotation2d(0));
            public static final Pose2d RedCage3 = new Pose2d(0,0, new Rotation2d(0));
            
            public static final Pose2d BlueCage1 = new Pose2d(0,0, new Rotation2d(0));
            public static final Pose2d BlueCage2 = new Pose2d(0,0, new Rotation2d(0));
            public static final Pose2d BlueCage3 = new Pose2d(0,0, new Rotation2d(0));
        }

        public static final class ReefConstants{

            public enum ReefPoleSide {

                //BLUE SIDE, should be mirrored for red side
                LEFT(new Pose2d[]{ //these are wrong obvi
                    new Pose2d(0.0, 0.0, new Rotation2d(0.0)), // Point A
                    new Pose2d(0.0, 1.0, new Rotation2d(0.0)), // Point C
                    new Pose2d(0.0, 2.0, new Rotation2d(0.0)), // Point E
                    new Pose2d(0.0, 3.0, new Rotation2d(0.0)), // Point G
                    new Pose2d(0.0, 4.0, new Rotation2d(0.0)), // Point I
                    new Pose2d(0.0, 5.0, new Rotation2d(0.0))  // Point K
                }),

                RIGHT(new Pose2d[]{
                    new Pose2d(1.0, 0.0, new Rotation2d(0.0)), // Point B
                    new Pose2d(1.0, 1.0, new Rotation2d(0.0)), // Point D
                    new Pose2d(1.0, 2.0, new Rotation2d(0.0)), // Point F
                    new Pose2d(1.0, 3.0, new Rotation2d(0.0)), // Point H
                    new Pose2d(1.0, 4.0, new Rotation2d(0.0)), // Point J
                    new Pose2d(1.0, 5.0, new Rotation2d(0.0))  // Point L
                });

                private final Pose2d[] waypoints;

                ReefPoleSide(Pose2d[] poses) {
                    this.waypoints = poses;
                }

                public Pose2d[] getPoints(ReefPoleSide side) {
                    return side.waypoints;
                }

                public Pose2d getClosestPoint(Pose2d robotPose) {
                    return Arrays.stream(this.waypoints)
                        .min(Comparator.comparingDouble(
                            point -> point.getTranslation().getDistance(robotPose.getTranslation())))
                        .orElse(null);
                }
            }

            public enum ReefPoleLevel { //We can also leave these empty and just use for display
                L1(0.0),
                L2(0.0),
                L3(0.0); //wont be using l4

                private final double elevatorLevel;

                ReefPoleLevel(double height) {
                    this.elevatorLevel = height;
                }

                public ReefPoleLevel raiseLevel() {
                    if(this.ordinal() == 2)
                        return this;
                    else
                        SmartDashboard.putNumber("Selected Reef Level", this.ordinal() + 1);
                        return ReefPoleLevel.values()[this.ordinal() + 1];
                }

                public ReefPoleLevel decreaseLevel() {
                    if(this.ordinal() == 0)
                        return this;
                    else
                        SmartDashboard.putNumber("Selected Reef Level", this.ordinal() - 1);
                        return ReefPoleLevel.values()[this.ordinal() - 1];
                }

                public double getElevatorLevel() {
                    return this.elevatorLevel;
                }
            }

        }

        
    }

    public static final double FIELD_WIDTH_METERS = 8.21055;
    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final double openLoopRamp = 0.25;

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}
