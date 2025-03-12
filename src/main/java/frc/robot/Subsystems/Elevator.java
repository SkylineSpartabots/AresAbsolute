// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private static Elevator instance;

  private TalonFX follower;
  private TalonFX leader;
  private VoltageOut voltOutput;
  private TorqueCurrentFOC torqueOutput;

  private boolean holdPosition = false;


  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public enum ElevatorState {
    L1(15), //bad
    L2(20.287),
    L3(35.29394),
    L4(61.17),
    GROUND(0.11), //bad
    A1(7.3), //bad
    A2(22), //bad
    SOURCE(0.5);
    //48.1 should be max
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
    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);
    configMotor(leader, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
    configMotor(follower, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);

    // follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));

    voltOutput = new VoltageOut(0).withEnableFOC(true);
    torqueOutput = new TorqueCurrentFOC(0);

  }

  private void configMotor(TalonFX motor, InvertedValue direction, NeutralModeValue neutralMode){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.elevatorContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.elevatorPeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits = currentLimitsConfigs;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motor.getConfigurator().apply(config);
    motor.getPosition().setUpdateFrequency(50);
    motor.getStatorCurrent().setUpdateFrequency(50);
    motor.getMotorVoltage().setUpdateFrequency(50);
    motor.optimizeBusUtilization();

    Slot0Configs configuration = new Slot0Configs();
    configuration.kG = 0.4;
    configuration.kI = 0.15;
    configuration.kP = 0.13;

    motor.getConfigurator().apply(configuration);
     // motor.optimizeBusUtilization();
  }

  public void setPosition(double position){
    leader.setControl(new PositionVoltage(position).withSlot(0).withUpdateFreqHz(50));
    follower.setControl(new PositionVoltage(position).withSlot(0).withUpdateFreqHz(50));
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public void setTorqueOutput(double output){
    leader.setControl(torqueOutput.withOutput(output));
    follower.setControl(torqueOutput.withOutput(output));
  }

  public double getCurrent(){
    return leader.getStatorCurrent().getValueAsDouble();
  }

  public void stop(){
    leader.setControl(voltOutput.withOutput(0));
    follower.setControl(voltOutput.withOutput(0));
  }

  public void setSpeed(double speed){
    holdPosition = false;
    leader.set(speed);
    follower.set(speed);
  }

  public void setVoltage(double voltage){
    holdPosition = false;
    leader.setControl(voltOutput.withOutput(voltage));
    follower.setControl(voltOutput.withOutput(voltage));
  }

  public double getVelocity(){
    return leader.getVelocity().getValueAsDouble();
  }

  public double getLeaderVoltage(){
    return leader.getMotorVoltage().getValueAsDouble();
  }

  public double getFollowerVoltage(){
    return follower.getMotorVoltage().getValueAsDouble();
  }

  public double getAcceleration() {
    return leader.getAcceleration().getValueAsDouble();
  }

  public void zeroPosition() {
    leader.setPosition(0);
    follower.setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("elevator position", getPosition());
    // SmartDashboard.putNumber("elevator stator current", getCurrent());
  }
}
