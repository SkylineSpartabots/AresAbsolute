// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Intake.PivotState;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.RollerState;
import frc.robot.commands.CancelableCommand;
import frc.robot.commands.DriveToPose;
// import frc.robot.commands.DriveToPose;
//import frc.robot.commands.CancelableCommand;
import frc.robot.commands.SetIntakePivot;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleLevel;
import frc.robot.RobotState.RobotState;

/** Add your docs here. */
public class CommandFactory {

    public static Command Intake(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetIntakePivot(PivotState.DOWN),
                new InstantCommand(()-> Intake.getInstance().setRollerSpeed(RollerState.INTAKE.getRollerSpeed()))
                )
            
        );
    }

    //add all mechanism off functions as they are tested; currently only pivot
    public static Command OffEverything() {
        return new ParallelCommandGroup(
            new SetIntakePivot(PivotState.UP),
            new InstantCommand(()-> Intake.getInstance().setRollerSpeed(0))
        );
    }

    public static Command Lift() {
       return new ParallelCommandGroup(
                new SetIntakePivot(PivotState.UP),
                new InstantCommand(()-> Intake.getInstance().brakeRoller())
            );
    }

    public static Command AutoScoreCoral(ReefPoleLevel level, ReefPoleSide side, CommandXboxController controller){
        return new ParallelCommandGroup(
            //align elevator to reef level
            new DriveToPose(side)
        ).raceWith(new CancelableCommand(controller));
    }

    public static Command Outtake() {
        return new InstantCommand(()->Intake.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }
}
