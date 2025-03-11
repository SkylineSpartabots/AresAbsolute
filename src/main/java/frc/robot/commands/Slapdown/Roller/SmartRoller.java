// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slapdown.Roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Slapdown.RollerState;

public class SmartRoller extends Command {
    private Slapdown s_Slapdown;
    Timer timer = new Timer();

    public SmartRoller() {
        s_Slapdown = Slapdown.getInstance();
//    addRequirements(s_Slapdown);
    }

    @Override
    public void initialize() {
        timer.restart();
        s_Slapdown.setRollerSpeed(RollerState.INTAKE.getRollerSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        s_Slapdown.brakeRoller();
        System.out.println("SmartRoller Ended");
    }


    @Override
    public boolean isFinished() {
        return s_Slapdown.getRollerVelocity() < 0.2 && timer.hasElapsed(0.25);
    }
}
