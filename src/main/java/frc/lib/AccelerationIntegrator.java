package frc.lib;
import frc.lib.Interpolating.Geometry.ITwist2d;

import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

public class AccelerationIntegrator {

    private double prevXAccel = 0.0;
    private double prevYAccel = 0.0;

    private double xVelocity = 0;
    private double yVelocity = 0;

    private double lastTimestamp = 0;

    private double[] prevAccel = new double[2];
    
    private Drivetrain s_Swerve = Drivetrain.getInstance();

    public void initTimeStamp(double timestamp){
        lastTimestamp = timestamp;
    }

    private final double alpha_1 = 0.80;  // TODO tune

    public void lowPassFilter(double[] update) {
        update[0] = alpha_1 * update[0] + (1-alpha_1) * prevAccel[0];
        update[1] = alpha_1 * update[1] + (1-alpha_1) * prevAccel[1];
    }

    private final double alpha_2 = 0.80;  // TODO tune

    public void complimentaryFilter(double[] update, double[] wheelVelocity) {
        update[0] = alpha_2 * update[0] + (1-alpha_2) * wheelVelocity[0];
        update[1] = alpha_2 * update[1] + (1-alpha_2) * wheelVelocity[1];
    }

    // Trapezoidal integration for velocity estimation (LIKE A BOSS)
    private void integrateAccel(double xAccel, double yAccel, double currentTime) {
        // if(lastTimestamp == -1) lastTimestamp = currentTime;

        double dt = currentTime - lastTimestamp;
        xVelocity += 0.5 * (xAccel + prevXAccel) * dt;
        yVelocity += 0.5 * (yAccel + prevYAccel) * dt;

        // Update prev
        prevXAccel = xAccel;
        prevYAccel = yAccel;
        lastTimestamp = currentTime;
    }

    public ITwist2d update(double[] update) {

        lowPassFilter(update);

        prevAccel = update;

        integrateAccel(update[0], update[1], update[2]);

        if(s_Swerve.getAbsoluteWheelVelocity() < 0.001){ //tune this threshold as needed
            xVelocity = 0;
            yVelocity = 0;
        } 

        return new ITwist2d(xVelocity, yVelocity);
    }

    public ITwist2d update(double[] update, double[] wheelVelocity) {
        
        lowPassFilter(update);

        prevAccel = update;

        integrateAccel(update[0], update[1], update[2]);

        complimentaryFilter(update, wheelVelocity);

        if(Math.abs(s_Swerve.getAbsoluteWheelVelocity()) == 0){ //tune this threshold as needed
            xVelocity = 0;
            yVelocity = 0;
        } 

        return new ITwist2d(xVelocity, yVelocity);
    }
}