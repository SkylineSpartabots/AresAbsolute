// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
//Should be integrated into intake subsystem, but we don't have a mechanism for that yet so it's being kept separate for now
public class LaserCAN extends SubsystemBase {
  
  private double laserThreshold = 15; //depends on lasercan placement
  private LaserCan laser;
  private static LaserCAN instance;

  public static LaserCAN getInstance(){
    if(instance == null){
      instance = new LaserCAN();
    }
    return instance;
  }

  public LaserCAN() {
    laser = new LaserCan(Constants.HardwarePorts.laserID);
    configLaser();
  }

  private void configLaser(){
    try{
      laser.setRangingMode(RangingMode.SHORT);
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 0, 0)); 
    }
    catch(ConfigurationFailedException e){
      SmartDashboard.putBoolean("laser working", false);
    }
  }

  public Measurement getLaserMeasurement(){
    return laser.getMeasurement();
  }

  @Override
  public void periodic() {
    // if(getLaserMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
    //   SmartDashboard.putNumber("lasercan measurement", getLaserMeasurement().distance_mm);
    // }

  }
}