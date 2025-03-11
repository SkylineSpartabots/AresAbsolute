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
import org.littletonrobotics.junction.AutoLogOutput;

public class Funnel extends SubsystemBase {
    private static Funnel instance;

    private TalonFX roller;

    private FunnelState state;

    public static Funnel getInstance() {
        if (instance == null) {
            instance = new Funnel();
        }
        return instance;
    }

    public enum FunnelState {
        INTAKE(0.5),
        OFF(0);
        private double speed;

        private FunnelState(double speed) {
            this.speed = speed;
        }

        private double getSpeed() {
            return speed;
        }
    }


    public Funnel() {
        roller = new TalonFX(Constants.HardwarePorts.funnelID, "mechbus");
        configMotor(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);
    }

    public void configMotor(InvertedValue direction, NeutralModeValue neutralMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        roller.optimizeBusUtilization();
        roller.getVelocity().setUpdateFrequency(Constants.dtMs);
    
        roller.getConfigurator().apply(config);
    }

    public void setSpeed(double speed) {
        roller.set(speed);
    }

    public void setState(FunnelState desiredState) {
        state = desiredState;
        roller.set(state.getSpeed());
    }

    @AutoLogOutput(key = "Funnel/Velocity")
    public double getVelocity() {
        return roller.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
    }
}
