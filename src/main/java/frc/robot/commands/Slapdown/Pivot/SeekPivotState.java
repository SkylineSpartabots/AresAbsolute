// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown.Pivot;

import frc.robot.Constants;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class SeekPivotState extends Command { // Dont Use
    private Slapdown s_Slapdown;
    private double angleSetpoint;
    private double speed;
    private PivotState state;
    private PIDController controller = new PIDController(2.8, 0, 0);

    public SeekPivotState(PivotState state) {
        s_Slapdown = Slapdown.getInstance();
        angleSetpoint = state.getPosition();
        this.state = state;
    }

    @Override
    public void initialize() {
        if (state != PivotState.DOWN) {
            s_Slapdown.setPivotSpeed(-0.35);
        } else {
            s_Slapdown.setPivotSpeed(0.3);
        }
    }

    @Override
    public void execute() {
        s_Slapdown.setPivotVoltage(controller.calculate(s_Slapdown.getPivotPosition(), angleSetpoint));

    }

    @Override
    public void end(boolean interrupted) {
        s_Slapdown.brakePivot();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Slapdown.getPivotPosition() - angleSetpoint) < 0.1 || s_Slapdown.getPivotStatorCurrent() > Constants.intakePivotCurrentThreshold;
    }
}
