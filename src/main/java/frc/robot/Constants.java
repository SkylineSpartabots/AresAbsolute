// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.Comparator;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double intakePivotCurrentThreshold = 70;
    public static double MaxSpeed = 6; //can be lowered during testing
    public static double MaxAcceleration = 4; //can be lowered during testing
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static double MaxAngularVelocity = 3 * Math.PI;
    public static double robotMass = 58.9; //kg
    public static double MOI = 0.14782; //sum of kg * m^2 to center of rotation
    public static double CoF = 1; // coefficient of friction TODO get better one
    public static double wheelRadiusInches = 1.9125; //inches


    public static DCMotor motorConfig = new DCMotor(
            Constants.KrakenConstants.nominalVoltageVolts,
            Constants.KrakenConstants.stallTorqueNewtonMeters,
            Constants.KrakenConstants.stallCurrentAmps,
            Constants.KrakenConstants.freeCurrentAmps,
            Constants.KrakenConstants.freeSpeedRadPerSec,
            4);

    public static ModuleConfig moduleConfig = new ModuleConfig(
            Constants.wheelRadiusInches * 0.0254, //in to m
            Constants.MaxSpeed,
            Constants.CoF,
            motorConfig,
            Constants.KrakenConstants.driveCurrentLimitAmps,
            Constants.KrakenConstants.torqueLoss,
            4
    );

    
    public static class LaserCANConstants {
        public static final double discardDistanceMM = 30; // Distance for which results should be discarded. in mm
    }
    public static class LimelightConstants {
        public static final String cameraName = "limelight"; // required for all LimelightHelpers method calls - or pass a blank string if the name is the default (limelight)

        // Limelight positions relative to the robot. In METERS and DEGREES. Relative to the center of the bot, ie the pigeon i assume.
        // I am assuming that the positives of x is forward, y is right, and z is up. LL docs say 'side' not right. Lets hope that right is positive side!
        // TODO these are NOT REAL - measure and then add in values for camera positioning
        public static final double forward = 0; // Meters
        public static final double right = 0; // Meters
        public static final double up = 0; // Meters
        public static final double roll = 0; // Degrees
        public static final double pitch = 0; // Degrees
        public static final double yaw = 0; // Degrees

        public static final Transform3d limelightToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(forward), Units.inchesToMeters(right), Units.inchesToMeters(up)),
                new Rotation3d(Units.degreesToRadians(roll), Units.degreesToRadians(pitch), Units.degreesToRadians(yaw))
        );

    }

    public static RobotConfig config = new RobotConfig(
            Constants.robotMass,
            Constants.MOI,
            Constants.moduleConfig,
            Constants.moduleLocations.FL,
            Constants.moduleLocations.FR,
            Constants.moduleLocations.BL,
            Constants.moduleLocations.BR
    );

    public static final class moduleLocations {
        public static final Translation2d FL = new Translation2d(-13.5, 13.5);
        public static final Translation2d FR = new Translation2d(13.5, 13.5);
        public static final Translation2d BL = new Translation2d(-13.5, -13.5);
        public static final Translation2d BR = new Translation2d(13.5, -13.5);
    }

    ;

    public static final class KrakenConstants {
        public static final double nominalVoltageVolts = 9; //website says up to 24 volts idk man
        public static final double stallTorqueNewtonMeters = 7;
        public static final double stallCurrentAmps = 366;
        public static final double freeCurrentAmps = 2;
        public static final double freeSpeedRadPerSec = 5800;
        public static final double driveCurrentLimitAmps = 60;
        public static final double torqueLoss = 60;
    }

    public static final double dt = 0.02; // 3/4 of a rotation per second max angular velocity
    public static int dtMs = 20; //20ms


    public static final int timeOutMs = 10;

    public static final double elevatorMaxVelocity = 250;
    public static final double elevatorMaxAcceleration = 300;
    public static final double elevatorCurrentThreshold = 20;

    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;

    public static final class OuttakePhysicalConstants {
        public static final double outtakeRollerRadius = 0;
        public static final double outtakeOffsetMillimeters = 0; //distance between center of robot and PVC center of mass after exiting outtake in mm
    }


    public static final double stickDeadband = 0.09;
    public static final double triggerDeadzone = 0.2;


    public static final class CurrentLimits {
        public static final int outtakeContinuousCurrentLimit = 35;
        public static final int outtakePeakCurrentLimit = 65;

        public static final int slapdownContinuousCurrentLimit = 40;
        public static final int slapdownPeakCurrentLimit = 70;

        public static final int elevatorContinuousCurrentLimit = 60;
        public static final int elevatorPeakCurrentLimit = 120;
    }


    public static Mode deployMode = Mode.REAL;

    public static final double outtakeAngle = 35;

    public static final class TrajectoryConstants {
        public static final double maxAcceleration = 2.0;
        public static final double maxVelocity = 6.0;

        public static final double poseToleranceX = 0.02;
        public static final double poseToleranceY = 0.02;
        public static final double poseToleranceTheta = Math.PI / 50; // 6 degrees
    }

    public static final double elevatorContinuousCurrentLimit = 50;
    public static final double elevatorPeakCurrentLimit = 80;

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final class VisionConstants {
        public static final String FRCamera = "Camera_0";
        public static final String FLCamera = "Camera_1";
        public static final String elevatorCamera = "Camera_2";
        public static final int aprilTagMax = 22;
        public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet | unnecessary, we have photonvision's field layout import
        public static final double cameraRollOffset = Units.degreesToRadians(0);
        public static final double cameraPitchOffset = Units.degreesToRadians(0);
        public static final double cameraYawOffset = Units.degreesToRadians(0);
        public static final double backRightCameraHeight = Units.inchesToMeters(9.1);
        public static final double backRightCameraPitch = Units.degreesToRadians(30);

        public static final double centerCameraHeight = Units.inchesToMeters(10.15);
        public static final double centerCameraPitch = Units.degreesToRadians(15);

        public static final class VisionLimits {
            public static final double rotationLimit = Math.PI;
            public static final double reprojectionLimit = 0.1;
            public static final double normThreshold = 0.1;
            public static final double ambiguityLimit = 0.6;
        }

        public static final class AprilTags {
            //fill with april tags
        }
    }


    // hardware ports for all hardware components on the robot
    // these include CAN IDs, pneumatic hub ports, etc.

    public static final class robotPIDs {


        public static final class DriveToPosePID {
            public static final double driveP = 4.5;
            public static final double thetaP = 1.25;
        }

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
        public static final int beamPort = 0;

        public static final int outtakeID = 21;
        public static final int algaeID = 22;
        public static final int laserID = 23;

        public static final int elevatorLeaderId = 31;
        public static final int elevatorFollowerId = 32;

        public static final int slapdownRollerID = 43;
        public static final int slapdownLeaderID = 41;
        public static final int slapdownFollowerID = 42;

        public static final int funnelID = 51;
    }

    //change for next game
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2022RapidReact); //WHERE IS NEW FIELD?

    public static final class FieldConstants {

        public static final Pose2d Procceser = new Pose2d(0, 0, new Rotation2d(0));

        public static final class Cage { // idk what our cage mech will be but might not need this
            public static final Pose2d RedCage1 = new Pose2d(0, 0, new Rotation2d(0));
            public static final Pose2d RedCage2 = new Pose2d(0, 0, new Rotation2d(0));
            public static final Pose2d RedCage3 = new Pose2d(0, 0, new Rotation2d(0));

            public static final Pose2d BlueCage1 = new Pose2d(0, 0, new Rotation2d(0));
            public static final Pose2d BlueCage2 = new Pose2d(0, 0, new Rotation2d(0));
            public static final Pose2d BlueCage3 = new Pose2d(0, 0, new Rotation2d(0));
        }

        static double temp = 0.25;

        public static final class ReefConstants {

            public enum ReefPoleSide {

                //BLUE SIDE
                LEFT(new Pose2d[]{ //these are wrong obvi
                        new Pose2d(3.61677, 2.5978, Rotation2d.fromRadians(1.0483)), // Point A
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point C
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point E
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point G
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point I
                        new Pose2d(temp, temp, new Rotation2d(0.0))  // Point K
                }),

                RIGHT(new Pose2d[]{
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point B
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point D
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point F
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point H
                        new Pose2d(temp, temp, new Rotation2d(0.0)), // Point J
                        new Pose2d(temp, temp, new Rotation2d(0.0))  // Point L
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

            public enum SourceNumber {
                S1("S1"),
                S2("S2");
                private String name;

                private SourceNumber(String name) {
                    this.name = name;
                }
            }

            public enum ReefNumber {
                R1,
                R2,
                R3,
                R4,
                R5,
                R6,
                R7,
                R8,
                R9,
                R10,
                R11,
                R12
            }


            public enum ReefPoleLevel { // Unused
                L1(0.0),
                L2(0.0),
                L3(0.0),
                L4(0.0);

                private final double elevatorLevel;

                ReefPoleLevel(double height) {
                    this.elevatorLevel = height;
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
