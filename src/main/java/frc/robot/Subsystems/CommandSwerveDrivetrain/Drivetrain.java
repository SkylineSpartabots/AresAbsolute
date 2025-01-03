package frc.robot.Subsystems.CommandSwerveDrivetrain;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.lib.Interpolating.Geometry.ITwist2d;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.robotPIDs.HeadingControlPID;
import frc.robot.RobotState.RobotState;
// import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double lastTimeReset = -1;

    RobotState robotState;

    private static Drivetrain s_Swerve = TunerConstants.DriveTrain;

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    public static Drivetrain getInstance(){
        if(s_Swerve == null){
            s_Swerve = new Drivetrain(TunerConstants.DrivetrainConstants, 250, TunerConstants.FrontLeft,
            TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);  
            
        }
        return s_Swerve;
    }

    // private void limit() {
    //     for (SwerveModule module : Modules) {
    //         CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    //         configs.SupplyCurrentLimit = 20;
    //         configs.SupplyCurrentLimitEnable = true;
    //         configs.StatorCurrentLimit = 40;
    //         configs.StatorCurrentLimitEnable = true;
            

    //         module.getDriveMotor().getConfigurator().apply(configs);
    //         module.getSteerMotor().getConfigurator().apply(configs);
    //     }
    // }

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules); //look here for parent library methods
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // limit();
    }

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        //robotState = RobotState.getInstance();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // limit();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
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
        tareEverything();
    }

    public void resetOdoUtil(Pose2d pose){
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, pose);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public Pose2d getPose(){
        return s_Swerve.m_odometry.getEstimatedPosition();
    }

    public void resetOdo(Pose2d pose){
        resetOdoUtil(pose);
        resetOdoUtil(pose);
        resetOdoUtil(pose);
        robotState.reset(0.02, new IPose2d(pose), ITwist2d.identity());
    }

    public double getHeading() {
        return getPose().getRotation().getRadians();
    }

    /**
     * Returns the current x and y velocities from the wheel encoders
     * 
     * @return double[] {VelocityX, VelocityY}
     * 
     */
    public double[] getWheelVelocities(){
        double roughVel[] = { 0.0, 0.0 }; // x and y
        for(int i = 0; i < ModuleCount; i++){
            SwerveModuleState module = Modules[i].getCurrentState();

            roughVel[0] += module.speedMetersPerSecond * module.angle.getCos();
            roughVel[1] += module.speedMetersPerSecond * module.angle.getSin();
        }
        
        roughVel[0] /= 4;
        roughVel[1] /= 4;

        return roughVel;
    }

    public double getAbsoluteWheelVelocity(){
        double velocity = 0;
        for(int i = 0; i < ModuleCount; i++){
            velocity += Modules[i].getCurrentState().speedMetersPerSecond;
        }
        return velocity/4;
    }

    public void updateOdometryByVision(Pose3d estimatedPose){
        System.out.println("Pose received");
        if(estimatedPose != null){
            s_Swerve.m_odometry.addVisionMeasurement(estimatedPose.toPose2d(), Logger.getRealTimestamp()); //Timer.getFPGATimestamp()
        }
    }

    public void updateOdometryByVision(Optional<EstimatedRobotPose> estimatedPose){
        if(estimatedPose.isPresent()){
            s_Swerve.m_odometry.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds); 
        }
    }

    private Pose2d autoStartPose = new Pose2d(2.0, 2.0, new Rotation2d());

    public void setAutoStartPose(Pose2d pose){
        autoStartPose = pose;
    }



    @Override
    public void periodic() {
        // System.out.println(robotState);
        Pose2d currPose = getPose();
        
        if(robotState != null){
             robotState.odometryUpdate(m_odometry.getEstimatedPosition(), getWheelVelocities(), Timer.getFPGATimestamp());

            // ITranslation2d currFilteredPose = robotState.getLatestFilteredPose();

            // SmartDashboard.putNumber("FILT X", currFilteredPose.getX());
            // SmartDashboard.putNumber("FILT Y", currFilteredPose.getY());
        }else{
            robotState = RobotState.getInstance();
        }
        

        //allows driver to see if resetting worked
        // SmartDashboard.putBoolean("Odo Reset (last 5 sec)", lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
        SmartDashboard.putNumber("ODO X", currPose.getX());
        SmartDashboard.putNumber("ODO Y", currPose.getY());
         SmartDashboard.putNumber("ODO ROT", currPose.getRotation().getRadians());
        // SmartDashboard.putNumber("AUTO INIT X", autoStartPose.getX());
        // SmartDashboard.putNumber("AUTO INIT Y", autoStartPose.getY());
         SmartDashboard.putNumber("current heading", getHeading());
        // SmartDashboard.putNumber("DT Vel", robotAbsoluteVelocity());
//        Logger.recordOutput("Odo Reset (last 5 sec)", lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
        Logger.recordOutput("Swerve/ODO X", currPose.getX());
        Logger.recordOutput("Swerve/ODO Y", currPose.getY());
        Logger.recordOutput("Swerve/ODO ROT", currPose.getRotation().getRadians());
//        Logger.recordOutput("Swerve/AUTO INIT X", autoStartPose.getX());
//        Logger.recordOutput("Swerve/AUTO INIT Y", autoStartPose.getY());
        Logger.recordOutput("Swerve/CurrentHeading", getHeading());
//        Logger.recordOutput("Swerve/DT Vel", robotAbsoluteVelocity());

        for(int i = 0; i < ModuleCount; i++){
            // Logger.recordOutput("Swerve/DriveMotor" + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            //Logger.recordOutput("Swerve/CANcoder module " + i, Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("CANcoder position module " + i, Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("drive motor velocity mod " + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Angle motor velocity mod " + i, Modules[i].getSteerMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/CANCoder Position/Module " + i, Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            Logger.recordOutput("Swerve/DriveMotorVelocity/Module " + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/SteerMotorVelocity/Module " + i, Modules[i].getSteerMotor().getVelocity().getValueAsDouble());
        }
    }

}
