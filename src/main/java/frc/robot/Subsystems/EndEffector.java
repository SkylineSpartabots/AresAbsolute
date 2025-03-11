// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    private static EndEffector instance;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OuttakeProfiler;
import frc.robot.Constants;

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    public EndEffector() {
        coral = new TalonFX(Constants.HardwarePorts.outtakeID, "mechbus");
        algae = new TalonFX(Constants.HardwarePorts.algaeID);
        laserCAN = new LaserCan(Constants.HardwarePorts.laserID);

  private DigitalInput beam;
  private TalonFX coral;
  private TalonFX algae;
  private LaserCan aligner;

    public enum OuttakeState { // not used
        OFF(0),
        INDEX(0.3),
        OUTTAKE(0.6);
        private double speed;

  public EndEffector() {
    beam = new DigitalInput(Constants.HardwarePorts.endEffectorBeamPort);
    coral = new TalonFX(Constants.HardwarePorts.outtakeID);
    aligner = new LaserCan(Constants.HardwarePorts.laserID);
    configLaser();
    // config(roller, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake);
    algae = new TalonFX(Constants.HardwarePorts.algaeID);
    config(coral, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    config(algae, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

        public double getSpeed() {
            return speed;
        }
    }

  public enum OuttakeState{
    HOLD(0),
    INDEX(-0.2),
    SCORE(-0.2);
    private double speed;
    private OuttakeState(double speed){
      this.speed = speed;
    }
    private double getSpeed() { //
      return speed;
    }
  }

    public void setCoralSpeed(double speed) {
        coral.set(speed);
    }

    public void setAlgaeSpeed(double speed) {
        algae.set(speed);
    }

    public void stopCoral() {
        coral.set(0);
    }

  private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = direction;
    
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.NeutralMode = neutralMode;
    
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.outtakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.outtakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    motor.optimizeBusUtilization();
    // config.CurrentLimits = currentLimitsConfigs;
    // // motor.optimizeBusUtilization();
  }
  
  public boolean getBeamResult(){
    return beam.get();
  }

    public void stop() {
        stopCoral();
        stopAlgae();
    }

  public void setOuttakeSpeed(OuttakeState state){
    coral.set(state.getSpeed());
  }

  public void setAlgaeSpeed(double speed){
    algae.set(speed);
  }
 
  //returns tangential speed of rollers
  // public double getOutputSpeed(){
  //   return roller.getVelocity().getValueAsDouble()*Math.PI*Constants.OuttakePhysicalConstants.outtakeRollerRadius;
  // }
    

    @AutoLogOutput(key = "EndEffector/Coral/Position")
    public double getCoralPosition() {
        return coral.getPosition().getValueAsDouble();
    }
    SmartDashboard.putBoolean("beam break unbroken", getBeamResult());
  }
}
