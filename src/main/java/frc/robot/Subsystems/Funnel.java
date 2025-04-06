// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */
  private static Funnel instance;
  
  private TalonFX gril;
  private TalonFX boril;
  private DigitalInput beam;
  
  private FunnelState state;

  public static Funnel getInstance(){
    if(instance == null){
      instance = new Funnel();
    }
    return instance;
  }

  public enum FunnelState{
    INTAKING(0.4254),
    EJECT(-0.3),
    OFF(0);
    private double rollerSpeed;
    private FunnelState(double rollerSpeed){
      this.rollerSpeed = rollerSpeed;
    }
    private double getRollerSpeed(){
      return rollerSpeed;
    }
  }


  public Funnel() {
    beam = new DigitalInput(Constants.HardwarePorts.funnelBeamPort);
    gril = new TalonFX(Constants.HardwarePorts.funnelgirlID);
    boril = new TalonFX(Constants.HardwarePorts.funnelguyID);
    configMotor(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast, gril);
    configMotor(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast, boril);
  }

  public void configMotor(InvertedValue direction, NeutralModeValue neutralMode, TalonFX motor){
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = neutralMode;
    config.MotorOutput.Inverted = direction;

    motor.getConfigurator().apply(config);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.optimizeBusUtilization();
  }

  public void setSpeed(double speed){
    gril.set(speed);
    boril.set(speed);
  }

  public boolean getBeamResult(){
    return beam.get();
  }

  public void setState(FunnelState desiredState){
    state = desiredState;
    gril.set(state.getRollerSpeed());
    boril.set(state.getRollerSpeed());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("funnel current", gril.getSupplyCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("funnel beam", getBeamResult());
  }
}
