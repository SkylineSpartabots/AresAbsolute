package frc.robot.Subsystems.CommandSwerveDrivetrain;

import java.sql.Driver;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.lib.Interpolating.Geometry.ITwist2d;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Constants.robotPIDs.HeadingControlPID;
import frc.robot.RobotState.RobotState;
// import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.commands.CancelableCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double lastTimeReset = -1;

    RobotState robotState;
    Field2d field = new Field2d();
    

    private Pose2d autoStartPose = new Pose2d(2.0, 2.0, new Rotation2d());
    


    // private DriveControlSystems controlSystem  = new DriveControlSystems(); //only for trajectory following


    private static CommandSwerveDrivetrain s_Swerve;

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private PIDController xController = new PIDController(0, 0, 0);
    private PIDController yController = new PIDController(0, 0, 0);
    private PIDController thetaController = new PIDController(0, 0, 0);
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    
    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }



    public static CommandSwerveDrivetrain getInstance(){
        if(s_Swerve == null){
            s_Swerve = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, 250, TunerConstants.FrontLeft,
            TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);  
        }
        return s_Swerve;
    }
    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public void applyFieldSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        System.out.println(speeds.toString());

        setControl(
        new SwerveRequest.ApplyFieldSpeeds()
        .withSpeeds(speeds)
        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY()));
    }

    public void applyFieldSpeeds(ChassisSpeeds speeds) {
        System.out.println(speeds.toString());

        setControl(
        new SwerveRequest.ApplyFieldSpeeds()
        .withSpeeds(speeds));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetOdo(){ //not being used, drivetrain.seedFieldRelative() instead for field centric driving
        s_Swerve.seedFieldCentric();
        robotState.reset(0.02, IPose2d.identity());
        robotState.resetUKF(IPose2d.identity());
    }

    public void resetOdo(Pose2d pose){
        resetOdoUtil(pose);
        robotState.reset(0.02, new IPose2d(pose));
        robotState.resetUKF(new IPose2d(pose));
    }
    
    public void resetOdoUtil(Pose2d pose){ //IDK if this works as we want it to
        s_Swerve.resetPose(pose);
    }

    public void setAutoStartPose(Pose2d pose){
        autoStartPose = pose;
    }

    @AutoLogOutput(key = "Swerve/OdometryPose2d")
    public Pose2d getPose(){
        return s_Swerve.getState().Pose;
    }
    @AutoLogOutput(key = "Swerve/Odometry ROT")
    public double getHeading() {
        return getPose().getRotation().getRadians();
    }

    @AutoLogOutput(key = "Swerve/DesiredStates")
    public SwerveModuleState[] getDesiredStates(){
        return new SwerveModuleState[] {
                s_Swerve.getModule(0).getTargetState(),
                s_Swerve.getModule(1).getTargetState(),
                s_Swerve.getModule(2).getTargetState(),
                s_Swerve.getModule(3).getTargetState()
        };
    }
    
    @AutoLogOutput(key = "Swerve/ActualStates")
    public SwerveModuleState[] getActualStates(){
        return new SwerveModuleState[] {
            s_Swerve.getModule(0).getCurrentState(),
            s_Swerve.getModule(1).getCurrentState(),
            s_Swerve.getModule(2).getCurrentState(),
            s_Swerve.getModule(3).getCurrentState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        System.out.println(s_Swerve.getState().Speeds.toString());
        return s_Swerve.getState().Speeds;
    }
    
    /**
     * Returns the current x and y velocities from the wheel encoders
     * 
     * @return double[] {VelocityX, VelocityY}
     * 
     */
    public double[] getWheelVelocities(){
        double roughVel[] = { 0.0, 0.0 }; // x and y
        for(int i = 0; i < 4; i++){
            SwerveModuleState module = s_Swerve.getStateCopy().ModuleStates[i];

            roughVel[0] += module.speedMetersPerSecond * module.angle.getCos();
            roughVel[1] += module.speedMetersPerSecond * module.angle.getSin();
        }
        
        roughVel[0] /= 4;
        roughVel[1] /= 4;

        return roughVel;
    }

    public double getAbsoluteWheelVelocity(){
        double velocity = 0;
        for(int i = 0; i < 4; i++){
            velocity += s_Swerve.getStateCopy().ModuleStates[i].speedMetersPerSecond;
        }
        return velocity/4;
    }
    


    @Override
    public void periodic() {
        if(robotState != null){
             robotState.odometryUpdate(this.getState(), Timer.getFPGATimestamp());

            ITranslation2d currFilteredPose = robotState.getLatestFilteredPose();

            Logger.recordOutput("RobotState/FilteredPose X", currFilteredPose.getX());
            Logger.recordOutput("RobotState/FilteredPose Y", currFilteredPose.getY());
            
        }else{
            robotState = RobotState.getInstance();
        }

        field.setRobotPose(robotState.getCurrentPose2d()); 
        SmartDashboard.putData("RobotState/Field2d", field); // cant this be done in logger?
        
        Pose2d currentPose = getPose();
        Logger.recordOutput("Swerve/Odometry X", currentPose.getX());
        Logger.recordOutput("Swerve/Odometry Y", currentPose.getY());
        Logger.recordOutput("Swerve/Odometry ROT", currentPose.getRotation().getRadians());
        
        // SmartDashboard.putNumber("AUTO INIT X", autoStartPose.getX());
        // SmartDashboard.putNumber("AUTO INIT Y", autoStartPose.getY());

        // SmartDashboard.putNumber("DT Vel", robotAbsoluteVelocity());
        
        // Allows driver to see if resetting worked
        Logger.recordOutput("Swerve/Odo Reset (last 5 sec)", lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
    }

}
