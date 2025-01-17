// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OuttakeProfiler;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
  
  private static Outtake instance;

  private OuttakeProfiler outtakeProfiler;

  private TalonFX topRoller;
  private TalonFX botRoller;

  public static Outtake getInstance(){
    if(instance == null) instance = new Outtake();
    return instance;
  }

  public Outtake() {
    topRoller = new TalonFX(Constants.HardwarePorts.topOuttake);
    botRoller = new TalonFX(Constants.HardwarePorts.botOuttake);
    config(topRoller, false, NeutralModeValue.Brake);
    config(botRoller, false, NeutralModeValue.Brake);
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

  private void config(TalonFX motor, boolean inverted, NeutralModeValue neutralMode){
    motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.outtakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.outtakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitsConfigs;
    motor.optimizeBusUtilization();
  }

  public void setSpeed(double speed){
    topRoller.set(speed);
    botRoller.set(speed);
  }

  public void setTopRollerSpeed(double speed){
    topRoller.set(speed);
  }

  public void setBotRollerSpeed(double speed){
    botRoller.set(speed);
  }


  //returns tangential speed of rollers
  public double getOutputSpeed(){
    return (botRoller.getVelocity().getValueAsDouble()+topRoller.getVelocity().getValueAsDouble())*Math.PI*Constants.OuttakePhysicalConstants.outtakeRollerRadius;
  }
    

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Can score L1", outtakeProfiler.coralTrajAligned());
  }
}
