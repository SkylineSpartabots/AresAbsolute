// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OuttakeProfiler;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  
  private static EndEffector instance;

  // private OuttakeProfiler outtakeProfiler;

  private DigitalInput beam;
  private TalonFX coral;
  private TalonFX algae;
  private LaserCan leftLaser;
  private LaserCan rightLaser;
  public boolean aligned = false;

  public static EndEffector getInstance(){
    if(instance == null) instance = new EndEffector();
    return instance;
  }

  public EndEffector() {
    beam = new DigitalInput(Constants.HardwarePorts.endEffectorBeamPort);
    coral = new TalonFX(Constants.HardwarePorts.outtakeID);
    leftLaser = new LaserCan(Constants.HardwarePorts.laserID);
    rightLaser = new LaserCan(Constants.HardwarePorts.laser2ID);
    configLasers();

    // config(roller, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake);
    algae = new TalonFX(Constants.HardwarePorts.algaeID);
    config(coral, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    config(algae, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

  }

  public enum OuttakeState{
    HOLD(0),
    INDEX(-0.2),
    SCORE(-0.3);
    private double speed;
    private OuttakeState(double speed){
      this.speed = speed;
    }
    private double getSpeed() { //
      return speed;
    }
  }

   private void configLasers(){
    try{
      leftLaser.setRangingMode(RangingMode.SHORT);
      leftLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      leftLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4)); 
    }
    catch(ConfigurationFailedException e){
      SmartDashboard.putBoolean("laser working", false);
    }
    try{
      rightLaser.setRangingMode(RangingMode.SHORT);
      rightLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      rightLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4)); 
    }
    catch(ConfigurationFailedException e){
      SmartDashboard.putBoolean("laser working", false);
    }
  }
  public Measurement getRightLaserMeasurement() {
    return rightLaser.getMeasurement();
  }

  public Measurement getLeftLaserMeasurement(){
    return leftLaser.getMeasurement();
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
    motor.getConfigurator().apply(config);
    // config.CurrentLimits = currentLimitsConfigs;
    // // motor.optimizeBusUtilization();
  }
  
  public boolean getBeamResult(){
    return beam.get();
  }

  public void setOuttakeSpeed(double speed){
    coral.set(speed);
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
    

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Can score L1", outtakeProfiler.coralTrajAligned());
    if(getLeftLaserMeasurement() != null){
      if(getLeftLaserMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
        SmartDashboard.putNumber("leftlasercan measurement", getLeftLaserMeasurement().distance_mm);
        SmartDashboard.putBoolean("leftlasercan working", true);
      }else{
        SmartDashboard.putNumber("leftlasercan measurement", -1);
        SmartDashboard.putBoolean("leftlasercan working", false );
        SmartDashboard.putNumber("leftlasercan status", getLeftLaserMeasurement().status);
      }
    } else{
      SmartDashboard.putNumber("lasercan measurement", -1);
      SmartDashboard.putBoolean("lasercan working", false );
    }
    if(getRightLaserMeasurement() != null){
      if(getRightLaserMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
        SmartDashboard.putNumber("rightlasercan measurement", getRightLaserMeasurement().distance_mm);
        SmartDashboard.putBoolean("rightlasercan working", true);
      }else{
        SmartDashboard.putNumber("rightlasercan measurement", -1);
        SmartDashboard.putBoolean("rightlasercan working", false );
        SmartDashboard.putNumber("rightlasercan status", getRightLaserMeasurement().status);
      }
    }

    if(getLeftLaserMeasurement()!= null && getRightLaserMeasurement() != null){
      if(getRightLaserMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && getLeftLaserMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
        if(getLeftLaserMeasurement().distance_mm < 300 && getRightLaserMeasurement().distance_mm < 300){
          SmartDashboard.putBoolean("aligned", true);
          aligned = true;
        } else{
          SmartDashboard.putBoolean("aligned", false);
          aligned = false;
        }
        
      }else{
        SmartDashboard.putBoolean("aligned", false);
        aligned = false;
      }
    }else{
      SmartDashboard.putBoolean("aligned", false);
      aligned = false;
    }
   SmartDashboard.putBoolean("beam break unbroken", getBeamResult());
  }
}
