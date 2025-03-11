// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown.Pivot;

import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.PivotState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class SetPivotState extends Command {
    private Slapdown s_Slapdown;
    private double angleSetpoint;
    private PivotState state;
    private PIDController controller = new PIDController(4.1, 2.3, 0.3);

    public SetPivotState(PivotState state) {
        s_Slapdown = Slapdown.getInstance();
        angleSetpoint = state.getPosition();
        this.state = state;
        addRequirements(s_Slapdown);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Slapdown/Pivot/TargetState", state.toString());
        
    }

    @Override
    public void execute() {
        s_Slapdown.setPivotVoltage(controller.calculate(s_Slapdown.getPivotPosition(), angleSetpoint));
        Logger.recordOutput("Slapdown/Pivot/Running", true);
    }

    @Override
    public void end(boolean interrupted) {
        s_Slapdown.setPivotVoltage(0);

        if (state != PivotState.DOWN){
            s_Slapdown.brakePivot(state);}

        System.out.println("SetPivotState Ended");
        Logger.recordOutput("Slapdown/Pivot/Running", false);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Slapdown.getPivotPosition() - angleSetpoint) < 0.1;
    }
}
