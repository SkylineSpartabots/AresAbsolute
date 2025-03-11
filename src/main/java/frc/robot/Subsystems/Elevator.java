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
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {


    private TalonFX follower;
    private TalonFX leader;
    private VoltageOut voltOutput;
    private TorqueCurrentFOC torqueOutput;
    private DigitalInput beamBreak;
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public enum ElevatorState {
        GROUND(0.1),
        L1(15),
        L2(30),
        L3(45),
        L4(60),
        SOURCE(10);
        //48.1 should be max
        private double encoderPosition;

        private ElevatorState(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
            return encoderPosition;
        }
    }


    public Elevator() {
        leader = new TalonFX(Constants.HardwarePorts.elevatorLeaderId, "mechbus");
        follower = new TalonFX(Constants.HardwarePorts.elevatorFollowerId, "mechbus");
        follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));
        beamBreak = new DigitalInput(Constants.HardwarePorts.beamPort);
//    leader.setNeutralMode(NeutralModeValue.Brake);
//    follower.setNeutralMode(NeutralModeValue.Brake); // does this way actually work? if so we need to use this not the config in the config method

        configMotor(leader, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
        configMotor(follower, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);

        voltOutput = new VoltageOut(0).withEnableFOC(true);
        torqueOutput = new TorqueCurrentFOC(0);
    }

    private void configMotor(TalonFX motor, InvertedValue direction, NeutralModeValue neutralMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.MotorOutput.Inverted = direction;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.elevatorContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.elevatorPeakCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits = currentLimitsConfigs;

        motor.optimizeBusUtilization();

        motor.getPosition().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50);

        motor.getConfigurator().apply(config);
    }

    public void stop() {
        leader.set(0);
    }

    public void zeroPosition() {
        leader.setPosition(0);
    }

    public void setSpeed(double speed) {
        leader.set(speed);
    }

    public void setVoltage(double voltage) {
        leader.setControl(voltOutput.withOutput(voltage));
    }

    public void setTorqueOutput(double output) {
        leader.setControl(torqueOutput.withOutput(output));
    }

    @AutoLogOutput(key = "Elevator/StatorCurrent")
    public double getStatorCurrent() {
        return leader.getStatorCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/SupplyCurrent")
    public double getSupplyCurrent() {
        return leader.getSupplyCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/Position")
    public double getPosition() {
        return leader.getPosition().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/Velocity")
    public double getVelocity() {
        return leader.getVelocity().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/Acceleration")
    public double getAcceleration() {
        return leader.getAcceleration().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/LeaderVoltage")
    public double getVoltage() {
        return leader.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/FollowerVoltage")
    public double getFollowerVoltage() {
        return follower.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Elevator/BeamBroken")
    public boolean getBeamResult() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
    }
}
