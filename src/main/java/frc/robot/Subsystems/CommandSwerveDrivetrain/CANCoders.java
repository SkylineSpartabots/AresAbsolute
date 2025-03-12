// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CommandSwerveDrivetrain;


import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class CANCoders extends SubsystemBase {
  private CANcoder frontLeftEncoder;
  private CANcoder frontRightEncoder;
  private CANcoder backLeftEncoder;
  private CANcoder backRightEncoder;

  private static CANCoders instance;

  public static CANCoders getInstance(){
    if(instance == null){
      instance = new CANCoders();
    }
    return instance;
  }

  public CANCoders() {
    frontLeftEncoder = new CANcoder(3, "drivetrain");
    frontRightEncoder = new CANcoder(4, "drivetrain");
    backLeftEncoder = new CANcoder(5, "drivetrain");
    backRightEncoder = new CANcoder(6, "drivetrain");
  }

  public void getNewOffsets(){
    SmartDashboard.putNumber("front left cancoder offset", (Constants.frontLeftEncoderOffset - frontLeftEncoder.getAbsolutePosition().getValueAsDouble()));
    // System.out.println("front left cancoder offset: " + (Constants.frontLeftEncoderOffset + frontLeftEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("front right cancoder offset", (Constants.frontRightEncoderOffset - frontRightEncoder.getAbsolutePosition().getValueAsDouble()));
    // System.out.println("front right cancoder offset: " + (Constants.frontRightEncoderOffset + frontRightEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("back left cancoder offset", (Constants.backLeftEncoderOffset - backLeftEncoder.getAbsolutePosition().getValueAsDouble()));
    // System.out.println("back left cancoder offset: " + (Constants.backLeftEncoderOffset + backLeftEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("back right cancoder offset", (Constants.backRightEncoderOffset - backRightEncoder.getAbsolutePosition().getValueAsDouble()));
    // System.out.println("back right cancoder offset: " + (Constants.backRightEncoderOffset + backRightEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  @Override
  public void periodic() {
    getNewOffsets();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("front left cancoder position", frontLeftEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("front right cancoder position", frontRightEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("back left cancoder position", backLeftEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("back right cancoder position", backRightEncoder.getAbsolutePosition().getValueAsDouble());
  }
}
