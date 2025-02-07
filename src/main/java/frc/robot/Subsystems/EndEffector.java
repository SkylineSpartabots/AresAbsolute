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
import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OuttakeProfiler;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  
  private static EndEffector instance;

  // private OuttakeProfiler outtakeProfiler;

  private TalonFX coral;
  private TalonFX algae;

  public static EndEffector getInstance(){
    if(instance == null) instance = new EndEffector();
    return instance;
  }

  public EndEffector() {
    coral = new TalonFX(Constants.HardwarePorts.outtakeID);
    // config(roller, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake);
    algae = new TalonFX(Constants.HardwarePorts.algaeID);
    config(coral, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    config(algae, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

  }

  public enum OuttakeState{
    HOLD(0),
    INTAKE(0.3),
    MIDOUTTAKE(0.6),
    L4SCORE(0.7, 0.4);
    private double topSpeed;
    private double botSpeed;
    private OuttakeState(double topSpeed, double botSpeed){
      this.topSpeed = topSpeed;
      this.botSpeed = botSpeed;
    }
    private OuttakeState(double speed){
      this.topSpeed = speed;
      this.botSpeed = speed;
    }
  }

  private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = direction;
    
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.NeutralMode = neutralMode;
    
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.outtakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.outtakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // config.CurrentLimits = currentLimitsConfigs;
    // // motor.optimizeBusUtilization();
  }

  public void setOuttakeSpeed(double speed){
    coral.set(speed);
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
  }
}
