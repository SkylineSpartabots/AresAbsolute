// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class LEDs extends SubsystemBase {
  
  private static LEDs instance;

  // private AddressableLED statusLED;
  // private AddressableLEDBuffer statBuffer;
  private RobotState robotState;
  private Elevator ele;
  private EndEffector ee; 
  private CommandSwerveDrivetrain dt;
  private Pose2d pose;
  private PWM redPWM;
  private PWM greenPWM;
  private PWM bluePWM;


  private double smallestTranslation = Double.POSITIVE_INFINITY;
  private double currentTranslation = 10;

  // private LEDPattern red = LEDPattern.solid(Color.kRed);
  // private LEDPattern purple = LEDPattern.solid(Color.kDarkViolet);
  // private LEDPattern blue = LEDPattern.solid(Color.kBlue);
  // private LEDPattern green = LEDPattern.solid(Color.kGreen);


  public static LEDs getInstance(){
    if(instance == null){
      instance = new LEDs();
    }
    return instance;
  }

  private LEDs() {
    redPWM = new PWM(6);
    greenPWM = new PWM(7);
    bluePWM = new PWM(8);
    // statusLED = new AddressableLED(Constants.HardwarePorts.statusLEDPort);
    // statBuffer = new AddressableLEDBuffer(1);

    redPWM.setPulseTimeMicroseconds(0);
    greenPWM.setPulseTimeMicroseconds(4095);
    bluePWM.setPulseTimeMicroseconds(3000);
    robotState = RobotState.getInstance();
    ele = Elevator.getInstance();
    ee = EndEffector.getInstance();
    dt = CommandSwerveDrivetrain.getInstance();
    // configLED();
  }

  private void configLED(){

    // statusLED.setData(statBuffer);
    // statusLED.setLength(statBuffer.getLength());
    // statusLED.start();
  }

  public void updateTeleop(){
    if(robotState.autoAligning == true){
      // blue.applyTo(statBuffer);
    } else if(Math.abs(ele.getPosition()-ele.getState().getEncoderPosition()) > 0.07){
      // red.applyTo(statBuffer);
    } else if(ee.aligned == true){
      // green.applyTo(statBuffer);
    } else{
      // purple.applyTo(statBuffer);
    }
  }

  public void updateDisabled(){
    smallestTranslation = Double.POSITIVE_INFINITY;
    
    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.161869, 5.6220193099)) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.161869, 5.6220193099));
    }

    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.1618696594, 2.45280456542968)) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.1618696594, 2.45280456542968));
    }
    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.17588882446, 4.146969165)) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.17588882446, 4.146969165));
    }
    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.1618696594, 2.45280456542968).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180))) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.1618696594, 2.45280456542968).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180)));
    }
    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.161869, 5.6220193099).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180))) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.161869, 5.6220193099).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180)));
    }
    if(dt.getPose().getTranslation().getDistance(new Translation2d(7.17588882446, 4.146969165).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180))) < smallestTranslation){
      smallestTranslation = dt.getPose().getTranslation().getDistance(new Translation2d(7.17588882446, 4.146969165).rotateAround(new Translation2d(8.790802, 4.03224), Rotation2d.fromDegrees(180)));
    }

    if (smallestTranslation < 0.07){
      // green.applyTo(statBuffer);
    } else{
      // red.applyTo(statBuffer);
    }
  
  }


  @Override
  public void periodic() {
  }
}
