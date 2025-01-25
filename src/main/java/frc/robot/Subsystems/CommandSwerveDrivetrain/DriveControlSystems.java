package frc.robot.Subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import javax.xml.stream.events.DTD;

// import org.littletonrobotics.junction.Logger;

// import frc.robot.RobotState.RobotState;

public class DriveControlSystems {

    private boolean slipControlOn = false;
    private boolean headingControl = false;
    private boolean shooterMode = false;
    private boolean aligning = false;
    private double lastHeading = 0;


    private double goalHeading = 0.0;

    //need tune this 
    private double filterThreshold = 0.15;

    //MODIFIED implementation to convert rotations to heading by integration

    private static Timer timer = new Timer();
    private double prevTimestamp = 0.0;


    // Can tune
    private double deadbandFactor = 0.8; // higher is more linear joystick controls


    CommandSwerveDrivetrain drivetrain;
    // RobotState robotState;

    PIDController pidHeading = new PIDController(0, 0, 0);

    // arbitrary number havent tested yet
    SlewRateLimiter limit = new SlewRateLimiter(0.2);

    public DriveControlSystems() {  
        // robotState = RobotState.getInstance();
        drivetrain = CommandSwerveDrivetrain.getInstance();

        timer.start();
    }

    //interface with modules
    public SwerveModule getModule(int index) {
      return drivetrain.getModule(index);
    }   

     // =======---===[ ⚙ Joystick processing ]===---========
    public SwerveRequest drive(double driverLY, double driverLX, double driverRX){
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxAngularRate;

        SmartDashboard.putNumber("requested velocity x", driverLX);
        SmartDashboard.putNumber("requested velocity y", driverLY);
        // Logger.recordOutput("JoystickProcessing/RequestedX", driverLX);
        // Logger.recordOutput("JoystickProcessing/RequestedY", driverLY);

        ChassisSpeeds speeds = new ChassisSpeeds(driverLY, driverLX, driverRX);

        double[][] wheelFeedFwX = calculateFeedforward();

        driverRX = limit.calculate(driverRX);         //slew rate limit for smoother inputs

        //ignore changes that are too small (number subject to change this is just a test value)
        // CHANGE: clamping v1 for continuous straight driving
        if(Math.abs(driverRX) > 0.05 || timer.get() - prevTimestamp > 0.05) { headingIntegrator(driverRX); SmartDashboard.putNumber("Goal heading", goalHeading); }
    
        driverRX = calculateGoalHeading(goalHeading, drivetrain.getHeading());         //if it's not needed it's not applied


        return new SwerveRequest.FieldCentric().withVelocityX(driverLX).withVelocityY(driverLY).withRotationalRate(driverRX);
        // .withSpeeds(speeds)
        // .withWheelForceFeedforwardsX(wheelFeedFwX[0])
        //  .withWheelForceFeedforwardsY(wheelFeedFwX[1])
        //  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
        // .withDesaturateWheelSpeeds(true);
    }

    private double[] previousVelocities = new double[4]; // To store previous velocity for each module

    public double[][] calculateFeedforward() {
        double[][] wheelFeedFwX = new double[2][4];
        //TODO tune
        double Kv = 0.01;  // velocity gain
        double Ka = 0.01;  // acceleration gain
        double Kf = 0;  // friction gain

        for (int i = 0; i < 4; i++) {
            double currentVelocity = getModule(i).getCurrentState().speedMetersPerSecond;
            double angleComponent = Math.cos(getModule(i).getCurrentState().angle.getRadians());

            double X = Kv * currentVelocity * angleComponent
            + Ka * ((currentVelocity - previousVelocities[i]) / Constants.dt) * angleComponent //acceleration
            + Kf;

            wheelFeedFwX[0][i] = X;
        }

        for (int i = 0; i < 4; i++) {
            double currentVelocity = getModule(i).getCurrentState().speedMetersPerSecond;
            double angleComponent = Math.sin(getModule(i).getCurrentState().angle.getRadians());

            double Y = Kv * currentVelocity * angleComponent
            + Ka * ((currentVelocity - previousVelocities[i]) / Constants.dt) * angleComponent //acceleration
            + Kf;

            previousVelocities[i] = currentVelocity;

            wheelFeedFwX[1][i] = Y;
        }

        return wheelFeedFwX;
    }

    public double scaledDeadBand(double input) {
        if(Math.abs(input) < Constants.stickDeadband) 
            return 0;
        else
            return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    public double calculateGoalHeading(double goalHeading, double driverRx) {
        double headingDifference = Math.IEEEremainder(drivetrain.getHeading() - goalHeading, 2 * Math.PI);

        if (Math.abs(headingDifference) >= updateFilterThreshold(drivetrain.getAbsoluteWheelVelocity(), driverRx)) {
            return headingControl(drivetrain.getHeading());
        } else {
            SmartDashboard.putBoolean("Heading control active", false);
        }

        return driverRx; //passes correction value back to drive()
    }

    public double headingControl(double currHeading) {
        updateGains(drivetrain.getAbsoluteWheelVelocity());       //same pid as original heading control

        currHeading = pidHeading.calculate(currHeading, goalHeading);         //heading -> rotational rate

        SmartDashboard.putBoolean("Heading control active", true);

        return currHeading;
    }


    // (EXPERIMENTAL) numerical definite integration i foudn this online
    public void headingIntegrator(double driverRX) {
        //change in heading, change in time 
        double time = timer.get();
        double dt = time - prevTimestamp;
        prevTimestamp = time;

        goalHeading = Math.IEEEremainder(goalHeading + driverRX * dt, 2*Math.PI);
    }

    //kind of similar approach to updateGains - based on robot speed optimal values could be different
    public double updateFilterThreshold(double velocity, double turnRate) {
        return filterThreshold * (1 + Math.log1p(velocity/Constants.MaxSpeed) + Math.log1p(turnRate/Constants.MaxAngularRate));     //**MORE CONSERVATIVE** 
        //return filterThreshold * (1 + velocity/Constants.MaxSpeed + turnRate/Constants.MaxAngularRate);        **MORE AGGRESSIVE**
    }



    // =======---===[ ⚙ Heading control ]===---========
    // public double headingControl(double driverRX){ //TODO tune high and low PID values
    //     if (!pidHeading.atSetpoint()) {
    //         double velocity = drivetrain.robotWheelVelocity();
    //         updateGains(velocity);
            
    //         // driverRX = pidHeading.calculate(robotState.robotYaw(), lastHeading);
    //         SmartDashboard.putBoolean("headingON", true);
//    Logger.recordOutput("HeadingControl/Active", true);

    //     } else {
    //         SmartDashboard.putBoolean("headingON", false);
    //         SmartDashboard.putNumber("lastHeading", lastHeading);
//    Logger.recordOutput("HeadingControl/Active", false);
//    Logger.recordOutput("HeadingControl/LastHeading", lastHeading);
    //     }

    //     return driverRX;
    // } 
    // TODO fix this later bruh

    public void updateGains(double velocity) {
        double speedRatio = Math.abs(Constants.MaxSpeed/velocity); //velocity is from wheels so could be off
        speedRatio = Math.max(0, Math.min(1, speedRatio));
        //clamp between 0 and 1

        //can tune
        pidHeading.setPID(
            interpolate(Constants.robotPIDs.HeadingControlPID.lowP, Constants.robotPIDs.HeadingControlPID.highP, speedRatio), // P
            0, // I (we do not need I)
            interpolate(Constants.robotPIDs.HeadingControlPID.lowD, Constants.robotPIDs.HeadingControlPID.highD, speedRatio) // D
            ); 
    }

    public double interpolate(double lower, double upper, double scale) {
        return Interpolator.forDouble().interpolate(lower, upper, scale);
    }

    // =======---===[ ⚙ Slip Control ]===---========

    //useless bc our IMU gives poop values

    public Double[] slipControl(double currentVelocity) { 

    Double[] outputs = new Double[4]; // reset to null every call
        for (int i = 0; i < 4; i++) {  //4 is module count but i dont want to make a getter
        
        //gets the ratio between what the encoders think our velocity is and the real velocity
        double slipRatio;
        if(currentVelocity == 0) { slipRatio = 1; } else {
            slipRatio = ((getModule(i).getCurrentState().speedMetersPerSecond) / currentVelocity); 
        }
        SmartDashboard.putNumber("Module " + i + " slipratio", slipRatio);
        // Logger.recordOutput("SwerveModules/SlipRatios/Module " + i , slipRatio);
        //if over the upper or lower threshold save the value
        if (slipRatio > (Constants.slipThreshold + 1) || slipRatio < (1 - Constants.slipThreshold)) {
            outputs[i] = slipRatio;
        }
    } 

    return outputs;
    } // runs periodically as a default command

    public void slipCorrection(Double[] inputs) {
        // divides by slip factor, more aggressive if far above slip threshold 
        for (int i = 0; i < 4; i++) { //4 is module count but i dont want to make a getter

            if (inputs[i] != null) {
                TalonFX module = (TalonFX) drivetrain.getModule(i).getDriveMotor();
                
                module.set(module.get() *
                 (1 + (Math.signum(inputs[i] - 1)) * (inputs[i] - Constants.slipThreshold)) / Constants.slipFactor);
                //https://www.desmos.com/calculator/afe5omf92p how slipfactor changes slip aggression

                SmartDashboard.putBoolean("slipON", true);
                // Logger.recordOutput("SlipControl/Active", true);
            }  else {
                SmartDashboard.putBoolean("slipON", false);
                // Logger.recordOutput("SlipControl/Active", false);
            } 
            
        }
    }

    //toggling ----------------------------------------------------------------
    public void setLastHeading() {
        lastHeading = drivetrain.getPose().getRotation().getRadians(); 
    }

    public void toggleHeadingControl() {
        headingControl = !headingControl;
    }

    public void toggleAlignment() {
        aligning = !aligning;   
    }

    public void toggleSlipControl() {
        slipControlOn = !slipControlOn;
    }

    public void setHeadingTolerance() {
        pidHeading.setTolerance(0.1745); // 10 degrees in radians
    }


}
