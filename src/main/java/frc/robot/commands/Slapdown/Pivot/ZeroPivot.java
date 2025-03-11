// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown.Pivot;

import frc.robot.Subsystems.Slapdown;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ZeroPivot extends Command {
    private Slapdown s_Slapdown;

    public ZeroPivot() {
        s_Slapdown = Slapdown.getInstance();
        addRequirements(s_Slapdown);
    }

    @Override
    public void initialize() {
        s_Slapdown.setPivotSpeed(-0.3); // TODO tune number
        Logger.recordOutput("Slapdown/Zeroing", true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_Slapdown.setPivotSpeed(0);
        s_Slapdown.zeroPivotPosition();
        s_Slapdown.brakePivot();
        System.out.println("ZeroPivot Ended");
        Logger.recordOutput("Slapdown/Zeroing", false);

    }

    @Override
    public boolean isFinished() {
        return s_Slapdown.getPivotSupplyCurrent() > 10; // TODO tune number
    }
}
