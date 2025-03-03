// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Slapdown.PivotState;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Slapdown;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetOuttake;
import frc.robot.commands.EndEffector.SmartCoralIndex;
import frc.robot.commands.Funnel.SetFunnel;
import frc.robot.commands.Slapdown.SetRoller;
import frc.robot.commands.Slapdown.SetPivot;
import frc.robot.commands.Slapdown.SmartAlgaeIntake;
import frc.robot.commands.SwerveCommands.DriveToPose;
import frc.robot.commands.SwerveCommands.DriveToPoseElevator;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.Constants.FieldConstants.ReefConstants.SourceNumber;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefNumber;
import frc.robot.RobotState.RobotState;

/** Add your docs here. */
public class CommandFactory {

    static EndEffector ee = EndEffector.getInstance();

    public static Command Dealgaeify(){
        return new SequentialCommandGroup(
            new SetElevator(ElevatorState.A1),
            new InstantCommand(()->ee.setAlgaeSpeed(0.4)),
            Commands.waitSeconds(0.5),
            new InstantCommand(()->ee.setAlgaeSpeed(0))
        );
    }
    public static Command Dealgaeify(ElevatorState state){
        return new SequentialCommandGroup(
            new SetElevator(state),
            new InstantCommand(()->ee.setAlgaeSpeed(0.4)),
            Commands.waitSeconds(0.5),
            new InstantCommand(()->ee.setAlgaeSpeed(0))
        );
    }

    //add all mechanism off functions as they are tested; currently only pivot
    public static Command OffEverything() {
        return new ParallelCommandGroup(
            new SetPivot(PivotState.UP),
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
                new SetPivot(PivotState.UP),
                new InstantCommand(()-> Slapdown.getInstance().brakeRoller())
            );
    }

    public static Command SmartAlgeaIntake() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetPivot(PivotState.DOWN),
                new SmartAlgaeIntake()
            ),
            new SetPivot(PivotState.HOLD)
        );
    }

    public static Command AlgeaOuttake() {
        return new SequentialCommandGroup(
            new SetRoller(RollerState.OUTTAKE),
            new SetRoller(RollerState.OFF)
        );
    }

    public static Command CoralIntake(){
        return new ParallelCommandGroup(
            new SetFunnel(FunnelState.INTAKING),
            new SetElevator(ElevatorState.SOURCE)
        );
    }

    public static Command ShootCoral(){
        return new SequentialCommandGroup(
            new SetOuttake(OuttakeState.SCOREMID),
            new SetElevator(ElevatorState.SOURCE)
        );
    }

    public static Command SmartCoralOuttake(){
        return new SequentialCommandGroup(
            new SetOuttake(OuttakeState.SCOREMID),
            new SetElevator(ElevatorState.SOURCE)
        );
    }

    public static Command SmartCoralOuttake(ElevatorState state){
        return new SequentialCommandGroup(
            new SetElevator(state),
            new SetOuttake(OuttakeState.SCOREMID),
            new SetElevator(ElevatorState.SOURCE)
        );
    }

    

    public static Command FinishIntake(){
        return new ParallelCommandGroup(
            new SetFunnel(FunnelState.OFF),
            new SetOuttake(OuttakeState.INDEX)
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

    public static Command FullCoralIntake(){
        return new SequentialCommandGroup(
            new SetElevator(ElevatorState.SOURCE),
            new SmartCoralIntake()
        );
    }

    public static Command AutoScoreCoral(ElevatorState level, ReefPoleSide side, CommandXboxController controller){
        return new SequentialCommandGroup(
            new DriveToPose(side),
            new SetOuttake(level)
        ).raceWith(new CancelableCommand(controller));
    }

    public static Command Outtake() {
        return new InstantCommand(()->Slapdown.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }

    // public static Command AutoScorefromSource(ElevatorState level, SourceNumber source, ReefNumber reef){
    //     return new ParallelCommandGroup(
            
    //     );
    // }
}
