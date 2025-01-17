// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private TalonFX pivot;
  private TalonFX roller;
  
  private static Intake instance;

  public static Intake getInstance(){
    if(instance == null){
      instance = new Intake();
    }
    return instance;
  }

  public Intake() {
    pivot = new TalonFX(Constants.HardwarePorts.intakePivotID);
    roller = new TalonFX(Constants.HardwarePorts.intakeRollerID);
  }

  private void configMotor(TalonFX motor, NeutralModeValue neutralMode){
    motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.intakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.intakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitsConfigs;

    motor.optimizeBusUtilization();
  }

  public enum PivotState{
    UP(10), //arbitrary numbers for now
    DOWN(5);

    private double position;
    private PivotState(double position){
      this.position = position;
    }
    public double getPosition(){
      return position;
    }
  }

  public enum RollerState{
    INTAKE(0.5),
    OUTTAKE(-0.5),
    OFF(0);
    private double motorSpeed;
    private RollerState(double motorSpeed){
      this.motorSpeed = motorSpeed;
    }
    public double getRollerSpeed(){
      return motorSpeed;
    }
  }

  public void stopPivot(){
    pivot.set(0);
  }

  public double getPosition(){
    return pivot.getPosition().getValueAsDouble();
  }

  public double getRollerCurrent(){
    return roller.getStatorCurrent().getValueAsDouble();
  }

  public void setPivotVoltage(double voltage){
    pivot.setControl(new VoltageOut(voltage));
  }

  public void setRollerSpeed(double speed){
    pivot.set(speed);
  }

  public void setRollerVoltage(double voltage){
    roller.setControl(new VoltageOut(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake pivot position", getPosition());
  }
}
