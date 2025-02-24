// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetOuttake;
import frc.robot.commands.EndEffector.SmartCoralIndex;
import frc.robot.commands.Funnel.SetFunnel;
import frc.robot.commands.Pivot.SetSlapdownPivot;
import frc.robot.commands.Pivot.SmartAlgaeIntake;
import frc.robot.commands.Slapdown.SetRoller;
import frc.robot.commands.SwerveCommands.DriveToPose;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Constants.FieldConstants.ReefConstants.SourceNumber;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefNumber;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleLevel;
import frc.robot.RobotState.RobotState;

/** Add your docs here. */
public class CommandFactory {

    //add all mechanism off functions as they are tested; currently only pivot
    public static Command OffEverything() {
        return new ParallelCommandGroup(
            new SetSlapdownPivot(PivotState.UP),
            new InstantCommand(()-> Slapdown.getInstance().setRollerSpeed(0))
        );
    }

    public static Command AutoCommand() {
        return new ParallelCommandGroup(
            new FollowChoreoTrajectory("1meter")
        );
    }

    public static Command Lift() {
       return new ParallelCommandGroup(
                new SetSlapdownPivot(PivotState.UP),
                new InstantCommand(()-> Slapdown.getInstance().brakeRoller())
            );
    }

    public static Command SmartAlgeaIntake() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetSlapdownPivot(PivotState.DOWN),
                new SmartAlgaeIntake()
            ),
            new SetSlapdownPivot(PivotState.HOLD)
        );
    }

    public static Command SmartCoralIntake() {
        return new SequentialCommandGroup(    
            new ParallelCommandGroup(
                new SetFunnel(FunnelState.INTAKING),
                new SmartCoralIndex()
            ),
            new SetFunnel(FunnelState.OFF)
        );
    }

    public static Command AlgeaOuttake() {
        return new SequentialCommandGroup(
            new SetRoller(RollerState.OUTTAKE),
            new SetRoller(RollerState.OFF)
        );
    }

    

    public static Command AutoScoreCoral(ElevatorState level, ReefPoleSide side, CommandXboxController controller){
        return new ParallelCommandGroup(
            new SetElevator(level),
            new DriveToPose(side)
        ).raceWith(new CancelableCommand(controller));
    }

    public static Command Outtake() {
        return new InstantCommand(()->Slapdown.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }


    public static Command AutoScorefromSource(ReefPoleLevel level, SourceNumber source, ReefNumber reef){
        return new ParallelCommandGroup(
            
        );
    }
}
