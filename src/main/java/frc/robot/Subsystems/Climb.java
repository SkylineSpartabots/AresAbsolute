// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private static Climb instance;

  private TalonFX climb;

  public static Climb getInstance(){
    if(instance == null){
      instance = new Climb();
    }
    return instance;
  }
  public Climb() {
    climb = new TalonFX(Constants.HardwarePorts.climbID, "mechbus");
    configMotor(climb, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
  }

  private void configMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = direction;
    config.MotorOutput.NeutralMode = neutralMode;
  }

  private void setSpeed(double speed){
    climb.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
