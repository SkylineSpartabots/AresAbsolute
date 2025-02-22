// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  private DigitalInput beam;

  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public enum ElevatorState {
    GROUND(0.1),
    L1(12),
    L2(24),
    L3(36),
    L4(48),
    SOURCE(10);
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
    leader = new TalonFX(Constants.HardwarePorts.elevatorLeaderId, "mechbus");
    follower = new TalonFX(Constants.HardwarePorts.elevatorFollowerId, "mechbus");
    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);
    configMotor(leader, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
    configMotor(follower, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);

    follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));

    beam = new DigitalInput(Constants.HardwarePorts.beamPort);
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
    motor.getConfigurator().apply(config);
    motor.getPosition().setUpdateFrequency(50);
    motor.getStatorCurrent().setUpdateFrequency(50);
    motor.optimizeBusUtilization();
  
     // motor.optimizeBusUtilization();
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public void setTorqueOutput(double output){
    leader.setControl(torqueOutput.withOutput(output));
  }

  public double getCurrent(){
    return leader.getStatorCurrent().getValueAsDouble();
  }

  public void stop(){
    leader.set(0);
  }

  public void setSpeed(double speed){
    leader.set(speed);
  }
  


  public double getVelocity(){
    return leader.getVelocity().getValueAsDouble();
  }

  public double getFollowerVoltage(){
    return follower.getMotorVoltage().getValueAsDouble();
  }

  public double getAcceleration() {
    return leader.getAcceleration().getValueAsDouble();
  }

  public void zeroPosition() {
    leader.setPosition(0);
  }

  public void setVoltage(double voltage){
    leader.setControl(voltOutput.withOutput(voltage));
  }

  public boolean getBeamResult(){
    return beam.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", getPosition());
    SmartDashboard.putBoolean("beam break result", getBeamResult());
    SmartDashboard.putNumber("elevator stator current", getCurrent());
  }
}
