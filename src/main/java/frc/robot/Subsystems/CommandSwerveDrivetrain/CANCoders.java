// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CommandSwerveDrivetrain;


import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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

  @Override
  public void periodic() {
    Logger.recordOutput("FL CANCoder Position", frontLeftEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("FR CANCoder Position", frontRightEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("BL CANCoder Position", backLeftEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("BR CANCoder Position", backRightEncoder.getAbsolutePosition().getValueAsDouble());
  }
}
