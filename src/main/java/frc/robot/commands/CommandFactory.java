// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

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
import frc.robot.commands.SwerveCommands.PoleAlign;
import frc.robot.commands.SwerveCommands.ReefAlign;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.SourceNumber;
import frc.robot.RobotState.RobotState;
import frc.robot.commands.EndEffector.SmartCoralIntake;

/** Add your docs here. */
public class CommandFactory {

    static EndEffector ee = EndEffector.getInstance();

    public static Command Dealgaeify(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetElevator(() -> ElevatorState.A1),
                new InstantCommand(()->ee.setAlgaeSpeed(0.65))
            ),
            Commands.waitSeconds(0.7),
            new InstantCommand(()->ee.setAlgaeSpeed(0))
        );
    }
    
    public static Command Dealgaeify(ElevatorState state){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetElevator(() -> state),
                new InstantCommand(()->ee.setAlgaeSpeed(0.5))
            ),
            Commands.waitSeconds(4.8254),
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

    public static Command CoralIntake(){
        return new ParallelCommandGroup(
            new SetFunnel(FunnelState.INTAKING),
            new SetElevator(() -> ElevatorState.SOURCE)
        );
    }

    public static Command ShootCoral(){
        return new SequentialCommandGroup(
            new SetOuttake(OuttakeState.SCORE)
        );
    }

    public static Command EjectFunnel(){
        return new SequentialCommandGroup(
            new SetFunnel(FunnelState.EJECT),
            Commands.waitSeconds(0.5),
            new SetFunnel(FunnelState.OFF)

        );
    }

    public static Command SmartCoralOuttake(){
        return new SequentialCommandGroup(
            new SetOuttake(OuttakeState.SCORE)
            
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
            new SetElevator(ElevatorState.SOURCE.getEncoderPosition()),
            new SmartCoralIntake()
        );
    }

    public static Command Outtake() {
        return new InstantCommand(()->Slapdown.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }

    //Automation commands
    public static Command AutoScoreCoral(Supplier<ElevatorState> level, Supplier<ReefPoleScoringPoses> pole, CommandXboxController controller){
        return new SequentialCommandGroup(
            new ReefAlign(pole),
            new PoleAlign(level, pole)
        ).raceWith(new CancelableCommand(controller));
    }

    public static Command AutoScoreCoralCloes(Supplier<ElevatorState> level, Supplier<ReefPoleScoringPoses> pole, CommandXboxController controller){
        return new SequentialCommandGroup( //i could make logic to make this a reef align if the pole is far away but that would be a lot of work
            new PoleAlign(level, pole)
        ).raceWith(new CancelableCommand(controller));
    }

    // public static Command AutoRemoveAlgae(Supplier<ElevatorState> level, CommandXboxController controller){
    //     return new SequentialCommandGroup(
    //         new AutomatedAlgaeAction(level)
    //     ).raceWith(new CancelableCommand(controller));
    // }

    // public static Command AutoScorefromSource(ElevatorState level, SourceNumber source, ReefNumber reef){
    //     return new ParallelCommandGroup(
            
    //     );
    // }
}
