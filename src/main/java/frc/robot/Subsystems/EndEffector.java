// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    private static EndEffector instance;


    private TalonFX coral;
    private TalonFX algae;
    private LaserCan laserCAN;

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    public EndEffector() {
        coral = new TalonFX(Constants.HardwarePorts.outtakeID, "mechbus");
        algae = new TalonFX(Constants.HardwarePorts.algaeID);
        laserCAN = new LaserCan(Constants.HardwarePorts.laserID);

        config(coral, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
        config(algae, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
        configLaser(laserCAN);
    }

    public enum OuttakeState { // not used
        OFF(0),
        INDEX(0.3),
        OUTTAKE(0.6);
        private double speed;

        private OuttakeState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    private void configLaser(LaserCan aligner) {
        try {
            aligner.setRangingMode(RangingMode.SHORT);
            aligner.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            aligner.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));

            Logger.recordOutput("EndEffector/LaserCAN/Working", true);
        } catch (ConfigurationFailedException e) {
            Logger.recordOutput("EndEffector/LaserCAN/Working", false);
        }
    }


    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.MotorOutput.Inverted = direction;

        currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.outtakeContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.outtakePeakCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits = currentLimitsConfigs;

        motor.optimizeBusUtilization();

//    motor.getSupplyCurrent().setUpdateFrequency(Constants.dtMs);
//    motor.getStatorCurrent().setUpdateFrequency(Constants.dtMs);
        motor.getVelocity().setUpdateFrequency(Constants.dtMs);

        motor.getConfigurator().apply(config);

    }

    public void setCoralSpeed(double speed) {
        coral.set(speed);
    }

    public void setAlgaeSpeed(double speed) {
        algae.set(speed);
    }

    public void stopCoral() {
        coral.set(0);
    }

    public void stopAlgae() {
        algae.set(0);
    }

    public void stop() {
        stopCoral();
        stopAlgae();
    }

    @AutoLogOutput(key = "EndEffector/Coral/Velocity")
    public double getCoralVelocity() {
        return algae.getVelocity().getValueAsDouble();
    }

    @AutoLogOutput(key = "EndEffector/Coral/Position")
    public double getCoralPosition() {
        return coral.getPosition().getValueAsDouble();
    }

    @AutoLogOutput(key = "EndEffector/Algae/Velocity")
    public double getAlgaeVelocity() {
        return algae.getVelocity().getValueAsDouble();
    }

    /**
     * Returns the speed of the game piece being ejected by the roller.
     *
     * @return the tangential speed of the outtake roller.
     */
    @AutoLogOutput(key = "EndEffector/TangentialSpeed")
    public double getTangentialSpeed() {
        return coral.getVelocity().getValueAsDouble() * Math.PI * Constants.OuttakePhysicalConstants.outtakeRollerRadius;
    }


    @AutoLogOutput(key = "EndEffector/LaserCAN/Measurement")
    public Measurement getLaserMeasurement() {
        return laserCAN.getMeasurement();
    }

    @AutoLogOutput(key = "EndEffector/LaserCAN/Distance")
    public double getLaserDistance(){
        return getLaserMeasurement().distance_mm;
    }

    @AutoLogOutput(key = "EndEffector/LaserCAN/CoralDetected")
    public boolean isCoralDetected() {
        return getLaserMeasurement().distance_mm < Constants.LaserCANConstants.discardDistanceMM;
    }

    @Override
    public void periodic() {
        Measurement measurement = getLaserMeasurement();
        if (measurement != null) {
            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                Logger.recordOutput("EndEffector/LaserCAN/Measurement", measurement.distance_mm);
                Logger.recordOutput("EndEffector/LaserCAN/Working", true);
                
            } else {
                Logger.recordOutput("EndEffector/LaserCAN/Working", false);
                Logger.recordOutput("EndEffector/LaserCAN/Measurement", -1);
                Logger.recordOutput("EndEffector/LaserCAN/Status", measurement.status);
            }
        }
        
    }
}
