// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown.Roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.RollerState;

public class SetRollerState extends Command {
    private Slapdown s_Slapdown;
    private RollerState state;

    public SetRollerState(RollerState state) {
        s_Slapdown = Slapdown.getInstance();
        this.state = state;
    }

    @Override
    public void initialize() {
        s_Slapdown.setRollerSpeed(state.getRollerSpeed());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
