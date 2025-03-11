// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Slapdown extends SubsystemBase {

    private TalonFX leader;
    private TalonFX follower;
    private TalonFX roller;

//  private TorqueCurrentFOC torqueOutput;
  
    private static Slapdown instance;

    public static Slapdown getInstance() {
        if (instance == null) {
            instance = new Slapdown();
        }
        return instance;
    }

    public Slapdown() {
        leader = new TalonFX(Constants.HardwarePorts.slapdownLeaderID, "mechbus");
        follower = new TalonFX(Constants.HardwarePorts.slapdownFollowerID, "mechbus");
        roller = new TalonFX(Constants.HardwarePorts.slapdownRollerID, "mechbus");

        follower.setControl(new Follower(Constants.HardwarePorts.slapdownLeaderID, true));

        configPivot(leader, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
        configPivot(follower, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
        configRoller(roller, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

//    torqueOutput = new TorqueCurrentFOC(0);
    }

    private void configPivot(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.MotorOutput.Inverted = direction;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.slapdownContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.slapdownPeakCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits = currentLimitsConfigs;

        Slot1Configs position = new Slot1Configs();
        position.GravityType = GravityTypeValue.Arm_Cosine;
        position.kP = 7.2;
        position.kI = 2.3;
        position.kG = 1.3;

        motor.optimizeBusUtilization();

        motor.getStatorCurrent().setUpdateFrequency(Constants.dtMs);
        motor.getPosition().setUpdateFrequency(Constants.dtMs);
        motor.getSupplyCurrent().setUpdateFrequency(Constants.dtMs);

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(position);
    }

    private void configRoller(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.MotorOutput.Inverted = direction;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.slapdownContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.slapdownPeakCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits = currentLimitsConfigs;

        Slot1Configs position = new Slot1Configs();
        position.kP = 2.4;
        position.kG = 1;

        motor.optimizeBusUtilization();

        motor.getSupplyCurrent().setUpdateFrequency(Constants.dtMs);
        motor.getStatorCurrent().setUpdateFrequency(Constants.dtMs);
        motor.getVelocity().setUpdateFrequency(Constants.dtMs);

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(position);
    }

    public enum PivotState {
        UP(0),
        HOLD(0.06), //TODO Test if we actually need this or if we can just use UP state instead - ie UP doesnt push the ball out
        DOWN(0.8125);

        private double position;

        private PivotState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum RollerState {
        INTAKE(0.45),
        OUTTAKE(-0.35),
        OFF(0);
        private double motorSpeed;

        private RollerState(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }

        public double getRollerSpeed() {
            return motorSpeed;
        }
    }

    /**
     * Break Roller at its current position. Hold the current position.
     */
    public void brakeRoller() {
        roller.setControl(
                new PositionVoltage(roller.getPosition().getValueAsDouble())
                        .withLimitForwardMotion(true)
                        .withLimitReverseMotion(true)
                        .withEnableFOC(true)
                        .withSlot(1)
        );
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public void setRollerVoltage(double voltage) {
        roller.setControl(new VoltageOut(voltage));
    }

    @AutoLogOutput(key = "Slapdown/Roller/Velocity")
    public double getRollerVelocity() {
        return roller.getVelocity().getValueAsDouble();
    }

    @AutoLogOutput(key = "Slapdown/Roller/SupplyCurrent")
    public double getRollerSupplyCurrent() {
        return roller.getSupplyCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Slapdown/Roller/StatorCurrent")
    public double getRollerStatorCurrent() {
        return roller.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Break Pivot at its current position. Use Position Voltage to maintain state.
     */
    public void brakePivot() {
        leader.setControl(
                new PositionVoltage(leader.getPosition().getValueAsDouble())
                        .withLimitForwardMotion(true)
                        .withEnableFOC(true)
                        .withSlot(1)
        );
    }

    /**
     * Break Pivot at the specified state. Use Position Voltage to maintain state
     *
     * @param state State to maintain.
     */
    public void brakePivot(PivotState state) {
        leader.setControl(
                new PositionVoltage(state.getPosition())
                        .withLimitForwardMotion(true)
                        .withEnableFOC(true)
                        .withSlot(1)
        );
    }

    public void setPivotSpeed(double speed) {
        leader.set(speed);
    }

    public void zeroPivotPosition() {
        leader.setPosition(0);
    }

    public void setPivotVoltage(double voltage) {
        leader.setControl(new VoltageOut(voltage));
    }

    public void stopPivot() {
        leader.set(0);
    }

    public void stopRoller() {
        roller.set(0);
    }

    public void stop() {
        stopPivot();
        stopRoller();
    }

    @AutoLogOutput(key = "Slapdown/Pivot/Position")
    public double getPivotPosition() {
        return leader.getPosition().getValueAsDouble();
    }

    @AutoLogOutput(key = "Slapdown/Pivot/StatorCurrent")
    public double getPivotStatorCurrent() {
        return leader.getStatorCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Slapdown/Pivot/SupplyCurrent")
    public double getPivotSupplyCurrent() {
        return leader.getSupplyCurrent().getValueAsDouble();
    }


    @Override
    public void periodic() {
    }
}
