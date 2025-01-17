// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private static Elevator instance;

  private TalonFX follower;
  private TalonFX leader;

  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public enum ElevatorState {
    L1(0),
    L2(1),
    L3(2),
    L4(3);
    private double encoderPosition;
    private ElevatorState(double encoderPosition){
      this.encoderPosition = encoderPosition;
    }
    public double getEncoderPosition(){
      return encoderPosition;
    }
  }


  public Elevator() {
    leader = new TalonFX(Constants.HardwarePorts.elevatorLeaderId);
    follower = new TalonFX(Constants.HardwarePorts.elevatorFollowerId);
    configMotor(leader, false, NeutralModeValue.Brake);
    configMotor(follower, false, NeutralModeValue.Brake);

    follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));
  }

  private void configMotor(TalonFX motor, boolean inverted, NeutralModeValue neutralMode){
    motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.elevatorContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.elevatorPeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitsConfigs;

    motor.optimizeBusUtilization();
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public void stop(){
    leader.set(0);
  }

  public double getVelocity(){
    return leader.getVelocity().getValueAsDouble();
  }

  public void setVoltage(double voltage){
    leader.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", getPosition());
  }
}
