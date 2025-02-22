// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.LimelightConstants.roll;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slapdown extends SubsystemBase {
  
  private TalonFX leader;
  private TalonFX follower;
  private TalonFX roller;

  private TorqueCurrentFOC torqueOutput;
  
  private static Slapdown instance;

  PIDController maintainPivotController = new PIDController(2, 0, 0.1);

  public static Slapdown getInstance(){
    if(instance == null){
      instance = new Slapdown();
    }
    return instance;
  }

  public Slapdown() {
    leader = new TalonFX(Constants.HardwarePorts.slapdownLeaderID, "mechbus");
    follower = new TalonFX(Constants.HardwarePorts.slapdownFollowerID, "mechbus");
    roller = new TalonFX(Constants.HardwarePorts.slapdownRollerID, "mechbus");
    follower.setControl(new Follower(Constants.HardwarePorts.slapdownLeaderID, true));

    configPivot(leader, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    configPivot(follower, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    configRoller(roller, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

    torqueOutput = new TorqueCurrentFOC(0);
  }

  private void configPivot(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.slapdownContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.slapdownPeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = neutralMode;

    config.CurrentLimits = currentLimitsConfigs;

    Slot1Configs position = new Slot1Configs();
    position.GravityType = GravityTypeValue.Arm_Cosine;
    position.kP = 6;
    position.kI = 1.5;
    position.kG = 1;


    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(position);
    motor.getStatorCurrent().setUpdateFrequency(50);
    motor.getPosition().setUpdateFrequency(50);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.optimizeBusUtilization(); 
  }


  private void configRoller(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.slapdownContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.slapdownPeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = neutralMode;

    config.CurrentLimits = currentLimitsConfigs;

    Slot1Configs position = new Slot1Configs();
    position.kP = 1.4;
    position.kG = 1;

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(position);
  }

  public enum PivotState{
    UP(-0.145),
    HOLD(-0.095),
    DOWN(0.8125);

    private double position;
    private PivotState(double position){
      this.position = position;
    }
    public double getPosition(){
      return position; 
    }
  }

  public enum RollerState{
    INTAKE(0.5425),
    OUTTAKE(-0.5425),
    OFF(0);
    private double motorSpeed;
    private RollerState(double motorSpeed){
      this.motorSpeed = motorSpeed;
    }
    public double getRollerSpeed(){
      return motorSpeed;
    }
  }

  public void brakeRoller(){
    roller.setControl(
      new PositionVoltage(roller.getPosition().getValueAsDouble())
      .withLimitForwardMotion(true)
      .withLimitReverseMotion(true)
      .withEnableFOC(true)
      .withSlot(1)
      );
  }

  // public void testUnbrake(){
  //   roller.setControl(new VoltageOut(0.3));
  // }

  public void stopPivot(){
    leader.set(0);
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public double getPivotCurrent(){
    return leader.getStatorCurrent().getValueAsDouble();
  }

  public void brakePivot(){
    leader.setControl(
      new PositionVoltage(leader.getPosition().getValueAsDouble())
      .withLimitForwardMotion(true)
      .withEnableFOC(true)
      .withSlot(1)
      );
  }

  public void brakePivot(PivotState state){
    leader.setControl(
      new PositionVoltage(state.getPosition())
      .withLimitForwardMotion(true)
      .withEnableFOC(true)
      .withSlot(1)
      );
  }

  public void setPivotSpeed(double speed){
    leader.set(speed);
  }

  public void setPivotPosition(PivotState state){
    leader.setControl(new PositionVoltage(state.getPosition()));
  }

  public double getResistiveCurrent(){
    return roller.getSupplyCurrent().getValueAsDouble();
  }
  
  public void resetPivotPosition(){
    leader.setPosition(0);
  }

  public void setPivotVoltage(double voltage){
    leader.setControl(new VoltageOut(voltage));
  }

  public void setRollerSpeed(double speed){
    roller.set(speed);
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
