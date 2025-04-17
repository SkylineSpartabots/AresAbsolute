// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double frontLeftEncoderOffset = 0.1235351525;
    public static double frontRightEncoderOffset = 0.263916012125;
    public static double backLeftEncoderOffset = -0.422119105625;
    public static double backRightEncoderOffset = -0.3244628895;

    public static double MaxSpeed = 6; //can be lowered during testing
    public static double MaxAcceleration = 1.2542976; //can be lowered during testing
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static double MaxAngularVelocity = 2 * Math.PI;

    public static Pose2d blueSource1 = new Pose2d(1.191, 7.0298, Rotation2d.fromRadians(-0.939839628289));
    public static Pose2d blueSource2 = new Pose2d(1.1702, 0.9921, Rotation2d.fromRadians(0.9404398081));
    public static Pose2d redSource1 = blueSource1.rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180));
    public static Pose2d redSource2 = blueSource2.rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180));


    //all these are outdated but we are not using them anymore so its fine tbh
    public static double robotMass = 25; //kg
    public static double MOI = 2; //sum of kg * m^2 to center of rotation
    public static double CoF = 1.0; // coefficient of friction TODO get better one
    public static double wheelRadiusInches = 1.9125; //inches

    public static double intakePivotCurrentThreshold = 70;

    public static DCMotor motorConfig = new DCMotor(
        Constants.KrakenConstants.nominalVoltageVolts,
        Constants.KrakenConstants.stallTorqueNewtonMeters,
        Constants.KrakenConstants.stallCurrentAmps,
        Constants.KrakenConstants.freeCurrentAmps,
        Constants.KrakenConstants.freeSpeedRadPerSec,
    2);

    public static ModuleConfig moduleConfig = new ModuleConfig(
        Constants.wheelRadiusInches * 0.0254, //in to m
        Constants.MaxSpeed,
        Constants.CoF,
        motorConfig,
        5.90777778,
        Constants.KrakenConstants.driveCurrentLimitAmps,
        1
    );

    public static RobotConfig config = new RobotConfig(
        Constants.robotMass,
        Constants.MOI,
        Constants.moduleConfig,
        Constants.moduleLocations.FL,
        Constants.moduleLocations.FR,
        Constants.moduleLocations.BL,
        Constants.moduleLocations.BR
    );

    public static class LimelightConstants{
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

    public static final class moduleLocations {
        public static final Translation2d FL = new Translation2d(-13.5, 13.5);
        public static final Translation2d FR = new Translation2d(13.5, 13.5);
        public static final Translation2d BL = new Translation2d(-13.5, -13.5);
        public static final Translation2d BR = new Translation2d(13.5, -13.5);
    };

    public static final class KrakenConstants {
        public static final double nominalVoltageVolts = 24; //website says up to 24 volts idk man
        public static final double stallTorqueNewtonMeters = 7.09;
        public static final double stallCurrentAmps = 366;
        public static final double freeCurrentAmps = 2.32;
        public static final double freeSpeedRadPerSec = 607;
        public static final double driveCurrentLimitAmps = 60;
        public static final double torqueLoss = 60;
    }

    public static final double dt = 0.02; // 3/4 of a rotation per second max angular velocity
    
    public static final int timeOutMs = 10;

    public static final double elevatorMaxVelocity = 250;
    public static final double elevatorMaxAcceleration = 300;
    public static final double elevatorCurrentThreshold = 20;

    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;
    
    public static final class OuttakePhysicalConstants{
        public static final double outtakeRollerRadius = 0;
        public static final double outtakeOffsetMillimeters = 0; //distance between center of robot and PVC center of mass after exiting outtake in mm
    }

    public static final double stickDeadband = 0.075;
    public static final double triggerDeadzone = 0.2;

    public static final class CurrentLimits{
        public static final int outtakeContinuousCurrentLimit = 35;
        public static final int outtakePeakCurrentLimit = 65;
        
        public static final int slapdownContinuousCurrentLimit = 40;
        public static final int slapdownPeakCurrentLimit = 70;

        public static final int elevatorContinuousCurrentLimit = 60;
        public static final int elevatorPeakCurrentLimit = 120;
    }
    
    public static Alliance alliance = Alliance.Blue;

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
        /** Running on a real robot. */
        REAL,

        /** Running a simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class VisionConstants {
        
        /*
        ----------------- Orange Pi Names -----------------
        In case anyone ever needs this, here is each orange pi host name and its assigned static IP
        alpha: 10.29.76.11
        beta: 10.29.76.12
        gamma: 10.29.76.13
        delta: 10.29.76.14
        backup: 10.29.76.15 - just a hot swappable SD card with a backup image
         */
        
        
        /*
        ----------------- Physical Placements -----------------
        FRONT RIGHT: FR, Diggy
        FRONT LEFT: FL, Liggy
        FRONT RIGHT ANGLED: FRA, Abe
        BACK RIGHT: BR, Gretchen
        BACK LEFT: BL, Blake
        BACK CENTER: BC, Charles        
        
        
        In the code we use the physical location names or abbreviations to refer to the cameras.
        The nicknames are to keep track of which camera is which and what its calibration is
        
        Name the camera, calibrate it, place it on the bot, and write down where you placed it (above).
         */
        
        public static final class CameraNames{

            public static final String FrontRight = "Biggy";
            public static final String FrontLeft = "Liggy";
            public static final String FrontRightCenter = "Abe";
            public static final String BackRight = "Gretchen";
            public static final String BackLeft = "Blake"; 
            public static final String BackCenter = "Charles"; 
        }
        
        public static final class CameraTransforms{
            /*
            
            WPI Coordinate Space
                    +x
                    |
            +y ----------- -y
                    |
                    -x
             
            Onshape Coordinate Space
                    +y
                    |
            -x ----------- +x
                    |
                    -y
                    
             +z is up in both coordinate spaces
                    
                    
             */
            
            //Pitch is COUNTERCLOCKWISE about the Y axis
            //Yaw is COUNTERCLOCKWISE about the Z axis
            
            // The camera to robot transform is the transform from the camera to the robot in the camera's coordinate space
            // We are moving from the camera to the center of the robot using WPI coordinates
            
            public static final Transform3d FLcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.323), Units.inchesToMeters(-8.563), Units.inchesToMeters(-7.372)),
                    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-15),Units.degreesToRadians(0))); //so this guy is negative not pos (cause ccw)

            public static final Transform3d FRcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.323), Units.inchesToMeters(8.563), Units.inchesToMeters(-7.372)),
                    new Rotation3d(Units.degreesToRadians(-4),Units.degreesToRadians(-15),Units.degreesToRadians(0)));

            public static final Transform3d FCcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.882), Units.inchesToMeters(0.500), Units.inchesToMeters(-6.898)), 
                    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-15),Units.degreesToRadians(0)));

            // Mechancical Advantage Cameras are 30 deg from -x axis (WPI) 
            public static final Transform3d BLcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(10.720), Units.inchesToMeters(-11.412), Units.inchesToMeters(-8.350)),
                    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-61.8750),Units.degreesToRadians(120))); // MA angled 30 deg to the left of -x (WPI) thus 150 deg CCW

            public static final Transform3d BRcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(10.720), Units.inchesToMeters(11.412), Units.inchesToMeters(-8.350)),
                    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-61.8750),Units.degreesToRadians(-120))); // MA angled 30 deg to the right of -x (WPI) thus 150 deg CW 

            public static final Transform3d BCcameraToRobot = new Transform3d(
                    new Translation3d(Units.inchesToMeters(13.382), Units.inchesToMeters(0.500), Units.inchesToMeters(-6.898)),
                    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-35),Units.degreesToRadians(180)));
        }

        
        
        public static final int aprilTagMax = 22;
       
        public static final class VisionLimits {
        public static final double k_rotationLimit = Math.PI;
        public static final double k_velocityLimit = 6;
        public static final double k_reprojectionLimit = 0.3;
        public static final double k_normThreshold = 0.1;
        public static final double k_ambiguityLimit = 0.25;
        public static final double k_areaMinimum = 0.35; //TODO
        public static final double k_skewLimit = 0.35; //TODO
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

        public static final int statusLEDPort = 9;

        public static final int endEffectorBeamPort = 9;
        public static final int funnelBeamPort = 7;

        public static final int outtakeID = 21;
        public static final int algaeID = 22;
        public static final int laserID = 23;
        public static final int laser2ID = 24;

        public static final int elevatorLeaderId = 31;
        public static final int elevatorFollowerId = 32;

        public static final int slapdownRollerID = 43;
        public static final int slapdownLeaderID = 41;
        public static final int slapdownFollowerID = 42;

        public static final int funnelguyID = 51;
        public static final int funnelgirlID = 52;

        public static final int climbID = 61;
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

        static double temp = 100;

        public static final class ReefConstants{

            public static final Pose2d reefMiddleBlue = new Pose2d(4.486, 4.025, Rotation2d.fromDegrees(0)); // this is the middle of the reef field, used for alignment purposes, change as needed
            public static final Pose2d reefMiddleRed = new Pose2d(8.57 + 4.486, 4.025, Rotation2d.fromDegrees(0)); // this is the middle of the reef field, used for alignment purposes, change as needed

            public enum ReefSidePositions {
                //blue 
                POLE_1AB(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                POLE_2CD(new Pose2d(3.7057, 2.6695, Rotation2d.fromRadians(1.0472))), //
                POLE_3EF(new Pose2d(5.27304,2.6685, Rotation2d.fromRadians(2.0944))), //
                POLE_4GH(new Pose2d(6.0602, 4.0236, Rotation2d.fromRadians(3.14159))),  //
                POLE_5IJ(new Pose2d(5.274,5.3828, Rotation2d.fromRadians(-2.0944))), //
                POLE_6KL(new Pose2d(3.7057, 5.3828, Rotation2d.fromRadians(-1.0472))), //

                //red
                POLE_AB(new Pose2d(8.57 + 6.0602, 4.0236, Rotation2d.fromRadians(3.14159))),  //
                POLE_CD(new Pose2d(8.57 + 5.274,5.3828, Rotation2d.fromRadians(-2.0944))), //
                POLE_EF(new Pose2d(8.57 + 3.7057, 5.3828, Rotation2d.fromRadians(-1.0472))), //
                POLE_GH(new Pose2d(8.57 + 2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                POLE_IJ(new Pose2d(8.57 + 3.7057, 2.6695, Rotation2d.fromRadians(1.0472))), //
                POLE_KL(new Pose2d(8.57 + 5.27304,2.6685, Rotation2d.fromRadians(2.0944))); //

                private final Pose2d waypoints;

                public double getReefSideRating(Pose2d currentPose) {
                    int rating = 0;
                    int k = 0;

                    double targetDistance = currentPose.getTranslation().getDistance(this.getPose().getTranslation());

                    System.out.println("targe " + targetDistance);
                    System.out.println(Constants.alliance.name());
                    if(Constants.alliance != Alliance.Blue)
                        k += 6; //move up ReefSidePoles

                        for (int i = k; i <= k + 5; i++) {
                            if(currentPose.getTranslation().getDistance(ReefSidePositions.values()[i].getPose().getTranslation()) <= targetDistance) {
                                rating++;
                            }
                            System.out.println(currentPose.getTranslation().getDistance(ReefSidePositions.values()[i].getPose().getTranslation()));
                        }

                        System.out.println("rating " + rating);

                        switch(rating)  { //TODO tune
                            case 1:
                                return 0; // none needed
                            case 2:
                                return 0.8; // slight adjustment
                            case 3:
                                return 0.8; // slight adjustment
                            case 4:
                                return 30;// mid adjustment
                            case 5:
                                return 30;// mid adjustment
                            case 6:
                                return 65; // high adjustment
                            default:                                                            
                                return 0; 
                        }
                }

                ReefSidePositions(Pose2d poses) {
                    this.waypoints = poses;
                }

                public Pose2d getPose() {
                    return this.waypoints;
                }
            }

            public enum ReefAlgaeRemovalPoses {

                ALG_AB(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                ALG_CD(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                ALG_EF(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                ALG_GH(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                ALG_IJ(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))), //
                ALG_KL(new Pose2d(2.921, 4.0259, Rotation2d.fromRadians(0.0))); //

                private final Pose2d waypoints;

                ReefAlgaeRemovalPoses(Pose2d poses) {
                    this.waypoints = poses;
                }

                public Pose2d getPose() {
                    return this.waypoints;
                }
            }

            public enum ReefPoleScoringPoses {
                //Now 4.2545 cm offset (to the right)
                //quarter inch to the right poses
                POLE_1A(new Pose2d(3.2808, 4.2257, Rotation2d.fromRadians(0.0)), "R12"), //
                POLE_2B(new Pose2d(3.2808, 3.89584, Rotation2d.fromRadians(0.0)), "R11"), //
                POLE_3C(new Pose2d(3.69869, 3.05849, Rotation2d.fromRadians(1.0472)), "R10"), //
                POLE_4D(new Pose2d(3.98269, 2.89349, Rotation2d.fromRadians(1.0472)), "R9"), //
                POLE_5E(new Pose2d(4.92909, 2.86057, Rotation2d.fromRadians(2.0944)), "R8"), //
                POLE_6F(new Pose2d(5.21309, 3.02457, Rotation2d.fromRadians(2.0944)), "R7"), //
                POLE_7G(new Pose2d(5.7232, 3.82435, Rotation2d.fromRadians(3.14159)), "R6"), //
                POLE_8H(new Pose2d(5.7232, 4.15335, Rotation2d.fromRadians(3.14159)), "R5"), //
                POLE_9I(new Pose2d(5.29194, 4.9965, Rotation2d.fromRadians(-2.0944)), "R4"), //
                POLE_10J(new Pose2d(4.96994, 5.182, Rotation2d.fromRadians(-2.0944)), "R3"), //
                POLE_11K(new Pose2d(4.03964, 5.19128, Rotation2d.fromRadians(-1.0472)), "R2"), //
                POLE_12L(new Pose2d(3.75364, 5.02628, Rotation2d.fromRadians(-1.0472)), "R1"), //

                // red
                POLE_A(new Pose2d(14.2932, 3.82435, Rotation2d.fromRadians(3.14159)), "R12"), //
                POLE_B(new Pose2d(14.2932, 4.15335, Rotation2d.fromRadians(3.14159)), "R11"), //
                POLE_C(new Pose2d(13.86194, 4.9965, Rotation2d.fromRadians(-2.0944)), "R10"), //
                POLE_D(new Pose2d(13.53994, 5.182, Rotation2d.fromRadians(-2.0944)), "R9"), //
                POLE_E(new Pose2d(12.60964, 5.19128, Rotation2d.fromRadians(-1.0472)), "R8"), //
                POLE_F(new Pose2d(12.32364, 5.02628, Rotation2d.fromRadians(-1.0472)), "R7"), //
                POLE_G(new Pose2d(11.8508, 4.2257, Rotation2d.fromRadians(0.0)), "R6"), //
                POLE_H(new Pose2d(11.8508, 3.89584, Rotation2d.fromRadians(0.0)), "R5"), //
                POLE_I(new Pose2d(12.26869, 3.05849, Rotation2d.fromRadians(1.0472)), "R4"), //
                POLE_J(new Pose2d(12.55269, 2.89349, Rotation2d.fromRadians(1.0472)), "R3"), //
                POLE_K(new Pose2d(13.49909, 2.86057, Rotation2d.fromRadians(2.0944)), "R2"), //
                POLE_L(new Pose2d(13.78309, 3.02457, Rotation2d.fromRadians(2.0944)), "R1"); //
                
                private final Pose2d waypoints;
                private final String name;
                ReefPoleScoringPoses(Pose2d poses, String choreoName) {
                    this.waypoints = poses;
                    this.name = choreoName;
                }

                public Pose2d getPose() {
                    return this.waypoints;
                }
                
                public String getName(){
                    return this.name;
                }
            }

            public enum SourceNumber {
                S1("S1"),
                S2("S2");
                private String name;
                private SourceNumber(String name){
                    this.name = name;
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
