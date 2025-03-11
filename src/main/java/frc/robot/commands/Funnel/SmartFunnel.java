// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Funnel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Funnel.FunnelState;

public class SmartFunnel extends Command {
    private Funnel s_Funnel;
    private EndEffector s_EndEffector;
    Timer timer = new Timer(); // Maybe used later for timeout


    public SmartFunnel() {
        s_Funnel = Funnel.getInstance();
        s_EndEffector = EndEffector.getInstance();
        addRequirements(s_Funnel);
    }

    @Override
    public void initialize() {
        s_Funnel.setState(FunnelState.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        s_Funnel.setState(FunnelState.OFF);
        System.out.println("SmartCoralIntake Ended");
    }

    @Override
    public boolean isFinished() {
        return s_EndEffector.getLaserMeasurement().distance_mm < Constants.LaserCANConstants.discardDistanceMM;// 3 cm
    }
}
