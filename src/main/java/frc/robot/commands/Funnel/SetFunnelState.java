// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Funnel.FunnelState;

public class SetFunnelState extends Command {
    private Funnel s_Funnel;
    private FunnelState state;

    public SetFunnelState(FunnelState state) {
        s_Funnel = Funnel.getInstance();
        this.state = state;
        addRequirements(s_Funnel);
    }

    @Override
    public void initialize() {
        s_Funnel.setState(state);
    }

    public void end(boolean interrupted) {
        System.out.println("SetFunnelState Ended");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
