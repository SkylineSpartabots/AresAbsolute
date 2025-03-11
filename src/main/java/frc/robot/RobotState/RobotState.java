package frc.robot.RobotState;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.SensorUtils;
import frc.lib.Interpolating.Geometry.IChassisSpeeds;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.lib.Interpolating.Geometry.ITwist2d;
import frc.lib.Interpolating.Interpolable;
import frc.lib.Interpolating.IDouble;
import frc.lib.Interpolating.InterpolatingTreeMap;
import frc.lib.VisionOutput;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class RobotState { //will estimate pose with odometry and correct drift with vision
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private static SensorUtils accelFilter = new SensorUtils();
    private static SensorUtils angularVelocityFilter = new SensorUtils();
    
    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;

    private InterpolatingTreeMap<IDouble, IPose2d> odometryPoses;
	private InterpolatingTreeMap<IDouble, ITranslation2d> filteredPoses;
    private InterpolatingTreeMap<IDouble, ITwist2d> robotIMUVelocity;
    private InterpolatingTreeMap<IDouble, IChassisSpeeds> robotOdomVelocity;
    private InterpolatingTreeMap<IDouble, IDouble> robotIMUAngularVelocity;
    private InterpolatingTreeMap<IDouble, IDouble> robotOdomAngularVelocity;
    private InterpolatingTreeMap<IDouble, ITwist2d> robotAccelerations;
    private InterpolatingTreeMap<IDouble, IChassisSpeeds> filteredRobotVelocities;

    // Matrix<N2, N2> initialCovariance = MatBuilder.fill(Nat.N2(), Nat.N2(),
    // 0.001, 0.0,
    // 0.0, 0.001 );

    // private UnscentedKalmanFilter<N2, N2, N2> UKF;

    private static final double dt = 0.020;
    private static final int observationSize = 50; //how many poses we keep our tree

	private Optional<ITranslation2d> initialFieldToOdo = Optional.empty();
    private Optional<Double> prevOdomTimestamp = Optional.empty();

	private boolean inAuto = false; //need to configure with auto but we dont have an auto yet (lol)

    public RobotState() {

        reefPoleLevel = ElevatorState.L1; //default state (needs to be L1 - L4)
        
        if(Constants.alliance.equals(Alliance.Blue)) {
            reefPole = ReefPoleScoringPoses.POLE_1A;
        } else {
            reefPole = ReefPoleScoringPoses.POLE_A;
        }

        drivetrain = CommandSwerveDrivetrain.getInstance();
        pigeon = drivetrain.getPigeon2();  //getting the already constructed pigeon in swerve
        reset(0.02, IPose2d.identity()); //init
    }

    public void odometryUpdate(SwerveDriveState state, double timestamp) {

        updateSensors();

        Pose2d currentPose = state.Pose;

        if(!prevOdomTimestamp.isEmpty()) {
            //merge our velocities
            IChassisSpeeds OdomVelocity =
            getInterpolatedValue(odometryPoses, prevOdomTimestamp.get(), IPose2d.identity())
            .getVelocityBetween(new IPose2d(state.Pose), timestamp - prevOdomTimestamp.get());

            
            robotOdomVelocity.put(new IDouble(timestamp), OdomVelocity);
            odometryPoses.put(new IDouble(timestamp), new IPose2d(state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation()));
    
        // Logger.recordOutput("Accel", robotAcceleration.toMagnitude());
        Logger.recordOutput("Robot State/ODO velocity", OdomVelocity.toMagnitude());
        Logger.recordOutput("Kinematics/Swerve/DT velocity", robotVelocityVector());
        // Logger.recordOutput("Kinematics/Swerve/DT angular velocity", robotAngularMagnitude.toDouble());
        // Logger.recordOutput("Kinematics/Swerve/DT acceleration", robotAcceleration.toMagnitude());
        }

        prevOdomTimestamp = Optional.of(timestamp);
    }

    public void reset(double time, IPose2d initial_Pose2d) { //init the robot state
        odometryPoses = new InterpolatingTreeMap<>(observationSize);
        odometryPoses.put(new IDouble(time), initial_Pose2d);
        robotIMUVelocity = new InterpolatingTreeMap<>(observationSize);
        robotIMUVelocity.put(new IDouble(time), ITwist2d.identity());    
        robotOdomVelocity = new InterpolatingTreeMap<>(observationSize);
        robotOdomVelocity.put(new IDouble(time), IChassisSpeeds.identity());          
        robotAccelerations = new InterpolatingTreeMap<>(observationSize);
        robotAccelerations.put(new IDouble(time), ITwist2d.identity());
        robotOdomAngularVelocity = new InterpolatingTreeMap<>(observationSize);
        robotOdomAngularVelocity.put(new IDouble(time), new IDouble(0.0));
        robotIMUAngularVelocity = new InterpolatingTreeMap<>(observationSize);
        robotIMUAngularVelocity.put(new IDouble(time), new IDouble(0.0));
        filteredRobotVelocities = new InterpolatingTreeMap<>(observationSize);
        filteredRobotVelocities.put(new IDouble(time), IChassisSpeeds.identity());
    }

    // Automation state methods

  private ElevatorState reefPoleLevel;
  private ReefPoleScoringPoses reefPole;

    public void reefPoleSet7() {
        reefPole = ReefPoleScoringPoses.POLE_7G;
    }

    public void reefPoleSet1() {
        reefPole = ReefPoleScoringPoses.POLE_1A;
    }

    public void navigateReefPoleUp() {
        System.out.println("up ordinal" + reefPole.ordinal());

        if(Constants.alliance == Alliance.Blue) {

            if(reefPole.ordinal() == 11) {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.POLE_1A.name());
                reefPole = ReefPoleScoringPoses.POLE_1A;
            } else {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.values()[reefPole.ordinal() + 1].name());
                reefPole = ReefPoleScoringPoses.values()[reefPole.ordinal() + 1];
            }

        } else {

            if(reefPole.ordinal() == 23) {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.POLE_A.name());
                reefPole = ReefPoleScoringPoses.POLE_A;
            } else {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.values()[reefPole.ordinal() + 1].name());
                reefPole = ReefPoleScoringPoses.values()[reefPole.ordinal() + 1];
            }
        }
    }

    public void navigateReefPoleDown() {
        System.out.println("down ordinal" + reefPole.ordinal());

        if(Constants.alliance == Alliance.Blue) {

            if(reefPole.ordinal() == 0) {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.POLE_12L.name());
                reefPole = ReefPoleScoringPoses.POLE_12L;
            } else {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.values()[reefPole.ordinal() - 1].name());
                reefPole = ReefPoleScoringPoses.values()[reefPole.ordinal() - 1];
            }

        } else {

            if(reefPole.ordinal() == 12) {
                SmartDashboard.putString
                ("Selected Pole", ReefPoleScoringPoses.POLE_L.name());
                reefPole = ReefPoleScoringPoses.POLE_L;
            } else {
                SmartDashboard.putString("Selected Pole", ReefPoleScoringPoses.values()[reefPole.ordinal() - 1].name());
                reefPole = ReefPoleScoringPoses.values()[reefPole.ordinal() - 1];
            }

        }
    }

    public ReefPoleScoringPoses getSelectedReefPole() {
        return reefPole;
    }

    public void raisePoleLevel() {
        if(!(reefPoleLevel.ordinal() == 3)) {
        SmartDashboard.putString("Selected Pole Level", ElevatorState.values()[reefPoleLevel.ordinal() + 1].name());
        reefPoleLevel = ElevatorState.values()[reefPoleLevel.ordinal() + 1];
        System.out.println(reefPoleLevel.name());
        }
    }

    public void lowerPoleLevel() {
        if(!(reefPoleLevel.ordinal() == 0)) {
        SmartDashboard.putString("Selected Pole Level", ElevatorState.values()[reefPoleLevel.ordinal() - 1].name());
        reefPoleLevel = ElevatorState.values()[reefPoleLevel.ordinal() - 1];
        }
    }

    public ElevatorState getSelectedElevatorLevel() {
        return reefPoleLevel;
    }
    

        public synchronized <T extends Interpolable<T>> T getInterpolatedValue(InterpolatingTreeMap<IDouble, T> map, Double timestamp, T identity) {

        if (map.isEmpty())
            return identity;

        if (timestamp == null) 
            return map.get(map.lastKey());

        // Interpolate for the given timestamp
        return map.getInterpolated(new IDouble(timestamp));
        }

        public synchronized ITranslation2d getLatestFilteredPose() {
		    return getInterpolatedValue(filteredPoses, filteredPoses.lastKey().value, ITranslation2d.identity());
	    }

        public synchronized IChassisSpeeds getLatestFilteredVelocity() {
		    return filteredRobotVelocities.get(filteredRobotVelocities.lastKey());
	    }

        public synchronized IChassisSpeeds getRobotVelocity(double timestamp) {
		    return filteredRobotVelocities.get(new IDouble(timestamp));
	    }

        public synchronized ITwist2d getLatestIMURobotVelocity() { //DONT USE
		    return getInterpolatedValue(robotIMUVelocity, robotIMUVelocity.lastKey().value, ITwist2d.identity()); 
	    }

        public synchronized IChassisSpeeds getLatestOdomRobotVelocity() {
		    return getInterpolatedValue(robotOdomVelocity, robotOdomVelocity.lastKey().value, IChassisSpeeds.identity());
	    }

        public synchronized ITwist2d getIMURobotVelocity(double timestamp) { //DONT USE
		    return getInterpolatedValue(robotIMUVelocity, timestamp, ITwist2d.identity());
	    }

        public synchronized IChassisSpeeds getOdomRobotVelocity(double timestamp) {
		    return getInterpolatedValue(robotOdomVelocity, timestamp, IChassisSpeeds.identity());  
	    }

    
        /**
         * Gets signed velocity from integrated acceleration from filtered velocities
         *
         * @return double VelocityVector
         */
        public synchronized double robotVelocityVector() {
            IChassisSpeeds latestVelocity = getLatestFilteredVelocity();
            return Math.signum(Math.atan2(latestVelocity.getVy(), latestVelocity.getVx())) * latestVelocity.toMagnitude();
        }

        //// =======---===[ ⚙ Pigeon2.0  ]===---========

        public void updateSensors() {
            double[] newAccel = accelFilter.filterAcceleration(rawRobotAcceleration());
            double[] newAngularVelocity = angularVelocityFilter.filterAngularVelocity(robotAngularVelocityMagnitude());
            robotIMUAngularVelocity.put(new IDouble(newAngularVelocity[1]), new IDouble(newAngularVelocity[0]));
            robotAccelerations.put(new IDouble(newAccel[2]), new ITwist2d(newAccel[0], newAccel[1]));
        }

        public void updateSensors(double[] wheelVelocity) {
            double[] newAccel = accelFilter.filterAcceleration(rawRobotAcceleration());
            double[] newAngularVelocity = robotAngularVelocityMagnitude();
            robotIMUAngularVelocity.put(new IDouble(newAngularVelocity[1]), new IDouble(newAngularVelocity[0]));
            robotAccelerations.put(new IDouble(newAccel[2]), new ITwist2d(newAccel[0], newAccel[1]));

            SmartDashboard.putNumber("raw Accel X", newAccel[0]);
            SmartDashboard.putNumber("raw Accel Y", newAccel[1]);
            // Logger.recordOutput("RobotState/raw Accel X", newAccel[0]);
            // Logger.recordOutput("RobotState/raw Accel Y", newAccel[1]);
        }

        public Rotation2d robotYaw() {
            return new Rotation2d(pigeon.getYaw()/*.refresh()*/.getValue());
        }

        /**
         * Gets the robot angular velocities from the pigeon
         *
         * @return double[] {AngularX, AngularY, Timestamp}
         */
        public synchronized double[] robotAngularVelocities(){
            double angularX = pigeon.getAngularVelocityXDevice().getValueAsDouble();
            double angularY = pigeon.getAngularVelocityYDevice().getValueAsDouble();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {angularX, angularY, timestamp};
        }

        /**
         * Gets the robot angular magnitude from the pigeon
         *
         * @return double[] {AngularMagnitude, Timestamp}
         */
        public synchronized double[] robotAngularVelocityMagnitude(){
            double angularX = pigeon.getAngularVelocityXDevice().getValueAsDouble();
            double angularY = pigeon.getAngularVelocityYDevice().getValueAsDouble();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {(Math.hypot(angularX, angularY)), timestamp};
        }

        /**
         * Gets the robot accelerations from the pigeon
         *
         * @return double[] {AccelerationX, AccelerationY, Timestamp}
         */
        public synchronized double[] rawRobotAcceleration() {
            double accelerationX = (pigeon.getAccelerationX().getValueAsDouble() - pigeon.getGravityVectorX().getValueAsDouble()) * 9.80665;
            double accelerationY = (pigeon.getAccelerationY().getValueAsDouble() - pigeon.getGravityVectorY().getValueAsDouble()) * 9.80665;
            
            double timestamp = pigeon.getAccelerationX().getTimestamp().getTime();
            SmartDashboard.putNumber("current timestamp", timestamp);
            // Logger.recordOutput("RobotState/current timestamp", timestamp);
            return new double[] {accelerationX, accelerationY, timestamp};
        }
 }