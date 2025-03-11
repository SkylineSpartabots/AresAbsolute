// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.commands.CancelableCommand;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.Funnel.SetFunnelState;
import frc.robot.commands.Funnel.SmartFunnel;
import frc.robot.commands.Slapdown.Pivot.SetPivotState;
import frc.robot.commands.Slapdown.Roller.SmartRoller;
import frc.robot.commands.Slapdown.Pivot.ZeroPivot;
import frc.robot.commands.SwerveCommands.DriveToPose;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Constants.FieldConstants.ReefConstants.SourceNumber;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefNumber;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleLevel;

public class CommandFactory {

    public static Command OffEverything() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> Slapdown.getInstance().stop()), // Stop roller and pivot motors
                new InstantCommand(() -> Elevator.getInstance().stop()), // Stop elevator motors
                new SetFunnelState(Funnel.FunnelState.OFF), // Stop funnel motor
                new InstantCommand(() -> EndEffector.getInstance().stop()) // stop algae and coral motors
                // TODO add climb stop when implemented

        );
    }

    public static Command ZeroAll() {
        return new ParallelCommandGroup(
                new ZeroElevator(),
                new ZeroPivot() // Do we need to zero pivot, i really dont wanna have to make certain that it stays up at start of match
        );
    }

    public static Command autoCommand() {
        return new ParallelCommandGroup(
                new FollowChoreoTrajectory("1meter")
        );
    }

    public static Command SmartAlgaeIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetPivotState(PivotState.DOWN),
                        new SmartRoller()
                ),
                new SetPivotState(PivotState.HOLD) // see if we can use UP instead of HOLD
        );

    }

    public static Command SmartCoralIntake() {
        return new ParallelCommandGroup(
                new SmartFunnel(),
                new SetElevator(ElevatorState.SOURCE)
        );
    }

    public static Command AutoScoreCoral(ElevatorState level, ReefPoleSide side, CommandXboxController controller) {
        return new ParallelCommandGroup(
                new SetElevator(level),
                new DriveToPose(side)
        ).raceWith(new CancelableCommand(controller));
    }

    public static Command AutoScorefromSource(ReefPoleLevel level, SourceNumber source, ReefNumber reef) {
        return new ParallelCommandGroup(

        );
    }
}
