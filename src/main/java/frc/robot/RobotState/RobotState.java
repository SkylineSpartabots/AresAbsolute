package frc.robot.RobotState;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.AccelerationIntegrator;
import frc.lib.VisionOutput;
import frc.lib.Interpolating.InterpolatingDouble;
import frc.lib.Interpolating.InterpolatingTreeMap;
import frc.lib.Interpolating.Geometry.InterpolablePose2d;
import frc.lib.Interpolating.Geometry.InterpolableTransform2d;
import frc.lib.Interpolating.Interpolable;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.Subsystems.Vision.Vision;
import org.opencv.video.KalmanFilter;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;


public class RobotState { //will estimate pose with odometry and correct drift with vision
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private static AccelerationIntegrator accelIntegrator = new AccelerationIntegrator();
    
    Drivetrain drivetrain;
    Pigeon2 pigeon = drivetrain.getPigeon2(); //getting the already constructed pigeon in swerve

    private InterpolatingTreeMap<InterpolatingDouble, InterpolablePose2d> odometryToVehicle;
	private InterpolatingTreeMap<InterpolatingDouble, InterpolableTransform2d> fieldToOdometry;
    private ExtendedKalmanFilter<N2, N2, N2> EKF;

    private static final double dt = 0.002;
    private static final int observationSize = 50; //how many poses we keep our tree
    private final static Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.05,0.05); // obtained from noise when sensor is at rest
    private final static Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.02,0.02); // idk how to find this but ill figure it out

	private Optional<InterpolableTransform2d> initialFieldToOdo = Optional.empty();
    private Optional<EstimatedRobotPose> prevVisionPose;

    private double velocityMagnitude;

	private boolean firstUpdate = false;
	private boolean inAuto = false;

    public RobotState() {
        drivetrain = Drivetrain.getInstance();
        initKalman();
        reset(0, InterpolablePose2d.identity());
    }

    public synchronized void visionUpdate(VisionOutput updatePose) {
        if (prevVisionPose.isEmpty() || initialFieldToOdo.isEmpty()) { //TODO make sure odo and vis pose are in same frame of ref
            double timestamp = updatePose.timestampSeconds;

            // merge poses
            //TODO this is wrong (i will fix later)
            InterpolableTransform2d visionTransform = new InterpolableTransform2d(updatePose.estimatedPose.getTranslation().toTranslation2d());
            InterpolableTransform2d proximateOdoTranslation = new InterpolableTransform2d(odometryToVehicle.getInterpolated(new InterpolatingDouble(timestamp)));
            InterpolableTransform2d mergedPose = visionTransform.translateBy(proximateOdoTranslation);
            fieldToOdometry.put(new InterpolatingDouble(timestamp),  mergedPose);
                
            //update kalman
            EKF.setXhat(0, mergedPose.getX());
            EKF.setXhat(1, mergedPose.getY());

            initialFieldToOdo = Optional.of(fieldToOdometry.lastEntry().getValue());
            prevVisionPose = Optional.ofNullable(updatePose); 
        } else {
            double timestamp = updatePose.timestampSeconds;
            InterpolablePose2d proximateOdoTranslation = odometryToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
            prevVisionPose = Optional.ofNullable(updatePose);
            InterpolableTransform2d visionTransform = new InterpolableTransform2d(prevVisionPose.get().estimatedPose.toPose2d());
            InterpolableTransform2d mergedPose = visionTransform.translateBy(new InterpolableTransform2d(proximateOdoTranslation));

            Vector<N2> stdevs = VecBuilder.fill(Math.pow(updatePose.getStandardDeviation(), 1), Math.pow(updatePose.getStandardDeviation(), 1));
					EKF.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									mergedPose.getX(),
									mergedPose.getY()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					fieldToOdometry.put(
							new InterpolatingDouble(timestamp),
							new InterpolableTransform2d(EKF.getXhat(0), EKF.getXhat(1)));
        } //my head hurts
    }


    //dont need any of the velocity values from odo bc our pigeon is prop more accurate than encoders
    public synchronized void odometryUpdate(Pose2d pose, double timestamp) {
        odometryToVehicle.put(new InterpolatingDouble(timestamp), new InterpolablePose2d(pose.getX(),pose.getY(), pose.getRotation()));

        // Keep the EKF ahead (no inputs)
		EKF.predict(VecBuilder.fill(0.0, 0.0), dt);
    }


    public void initKalman() {
        EKF = new ExtendedKalmanFilter<>(Nat.N2(), Nat.N2(), Nat.N2(),
        (x,u) -> u, // return input as the output (f)
        (x,u) -> x, // return states as the output (h)
        stateStdDevs, measurementStdDevs, dt);

        // states - A Nat representing the number of states.
        // outputs - A Nat representing the number of outputs.
        // f - A vector-valued function of x and u that returns the derivative of the state vector.
        // h - A vector-valued function of x and u that returns the measurement vector.
        // stateStdDevs - Standard deviations of model states.
        // measurementStdDevs - Standard deviations of measurements.
        // nominalDtSeconds - Nominal discretization timestep.

        /*  ExtendedKalmanFilter​(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<States,​N1>> f,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<Outputs,​N1>> h,
        Matrix<States,​N1> stateStdDevs, Matrix<Outputs,​N1> measurementStdDevs,
        double dtSeconds)
         */
        }


        public void reset(double time, InterpolablePose2d initial_Pose2d) { //basically init the robot state
            odometryToVehicle = new InterpolatingTreeMap<>(observationSize);
            odometryToVehicle.put(new InterpolatingDouble(time), initial_Pose2d);
            fieldToOdometry = new InterpolatingTreeMap<>(observationSize);
            // field_to_odometry.put(new InterpolatingDouble(time), getInitialFieldToOdom());
            prevVisionPose = Optional.empty();
        }


        // wawawawa

        public synchronized InterpolableTransform2d getInitialFieldToOdom() {
            if (initialFieldToOdo.isEmpty()) return InterpolableTransform2d.identity();
            return initialFieldToOdo.get();
        }


        public void updateAccel() {
            double[] newAccel = rawRobotAcceleration();
            velocityMagnitude = accelIntegrator.update(newAccel[0], newAccel[1]);
        }


        public double getVelocity() {
            return velocityMagnitude;
        }


        public double[] rawRobotAngularVelocity(){
            double angularX = pigeon.getAngularVelocityXDevice().getValue();
            double angularY = pigeon.getAngularVelocityYDevice().getValue();

            double timestamp = 0;
            
            return new double[] {(Math.signum(Math.atan2(angularY, angularX)) * Math.sqrt((Math.pow(angularX, 2)) + Math.pow(angularY, 2))), timestamp};
        }

    
        public double[] rawRobotAcceleration() {
            double accelerationX = pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue();
            double accelerationY = pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue();
            
            double timestamp = 0; //TODO how to get pigeon accurate timestamp help

            return new double[] {Math.signum(Math.atan2(accelerationX, accelerationY)) * Math.sqrt((Math.pow(accelerationX, 2)) + Math.pow(accelerationY, 2)), timestamp};
        }





        // -----------------------------------------------------------
                // BiFunction<Matrix<N3, N1>, Matrix<N2, N1>, Matrix<N3, N1>> f = (state, input) -> {
        //     double x = state.get(0, 0);
        //     double y = state.get(1, 0);
        //     double velx = state.get(2, 0);
        //     double vely = state.get(3, 0);
        
        //     double accelx = input.get(0, 0);
        //     double accely = input.get(1, 0);

        //     return VecBuilder.fill(
        //         x + velx * dt,                // New x position
        //         y + vely * dt,                // New y position
        //         vely + accely * dt                // New y velocity
        //         );
        // };

        // BiFunction<Matrix<N3, N1>, Matrix<N2, N1>, Matrix<N2, N1>> h = (stateEstimate, state) -> {
        //     double x = stateEstimate.get(0, 0);
        //     double y = stateEstimate.get(1, 0);
        //     double velx = stateEstimate.get(2, 0);
        //     double vely = stateEstimate.get(3, 0);

        //     return VecBuilder.fill(
        //         x,         //  x position
        //         y,         //  y position
        //         Math.hypot(velx, vely) // velocity magnitude
        //         );
        // };

        // --------- if we ever need a more complicated filter, you can implement it with the code above ------ 
 }