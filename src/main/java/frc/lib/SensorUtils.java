package frc.lib;
import frc.lib.Interpolating.Geometry.ITwist2d;

import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class SensorUtils {

    private double[] prevValue = new double[2];
    
    double owo = 0;
    double uwu = 0;
    double meow = 0;
    
    private CommandSwerveDrivetrain s_Swerve = CommandSwerveDrivetrain.getInstance();

    public void lowPassFilter(double[] value, double alpha) {
        value[0] = alpha * value[0] + (1-alpha) * prevValue[0];
        value[1] = alpha * value[1] + (1-alpha) * prevValue[1];
    }

        // meow++;
        // owo += value[0];
        // uwu += value[1];

        // System.out.println("X accel bias: " + owo/meow);
        // System.out.println("Y accel bias: " + uwu/meow);
        
        // X accel bias: 0.001175
        // Y accel bias: -0.00163

    public double[] filterAcceleration(double[] value) {

        lowPassFilter(value, 0.9);

        prevValue = value;

        //sensor bias
        value[0] -= 0.001175;
        value[1] -= -0.00163;

        if(s_Swerve.getAbsoluteWheelVelocity() < 0.00001){ //tune this threshold as needed
            double[] empty = {0,0, value[2]};
            return empty;
        } 

        return value;
    }

    public double[] filterAngularVelocity(double[] value) {
        lowPassFilter(value, 0.9);

        prevValue = value; 

        if(s_Swerve.getAbsoluteWheelVelocity() < 0.00001){ //tune this threshold as needed
            double[] empty = {0, value[1]};
            return empty;
        } 

        return value;
    }

}