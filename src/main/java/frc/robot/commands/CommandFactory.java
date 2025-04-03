// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.Subsystems.Slapdown.RollerState;
import frc.robot.commands.Autos.FollowChoreoTrajectory;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetAlgae;
import frc.robot.commands.EndEffector.SetOuttake;
import frc.robot.commands.EndEffector.SmartCoralIndex;
import frc.robot.commands.Funnel.SetFunnel;
import frc.robot.commands.Slapdown.SetRoller;
import frc.robot.commands.Slapdown.SetPivot;
import frc.robot.commands.Slapdown.SmartAlgaeIntake;
import frc.robot.commands.TeleopAutomation.DriveToPose;
import frc.robot.commands.TeleopAutomation.DriveToPoseChill;
import frc.robot.commands.TeleopAutomation.TeleopPathing;
import frc.robot.commands.TeleopAutomation.AlgaeAlign;
import frc.robot.commands.TeleopAutomation.PoleAlign;
import frc.robot.commands.TeleopAutomation.ReefAlign;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleScoringPoses;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSidePositions;
import frc.robot.Constants.FieldConstants.ReefConstants.SourceNumber;
import frc.robot.RobotState.RobotState;
import frc.robot.commands.EndEffector.SmartCoralIntake;

/** Add your docs here. */
public class CommandFactory {

    static EndEffector ee = EndEffector.getInstance();
    static CommandSwerveDrivetrain dt = CommandSwerveDrivetrain.getInstance();

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
        return new ParallelCommandGroup(
                new SetElevator(() -> state),
                new SetAlgae(0.5)
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
            new ParallelCommandGroup(
                new SetFunnel(FunnelState.EJECT),
                new InstantCommand(()->ee.setOuttakeSpeed(0.8))
            ),
            Commands.waitSeconds(0.5),
            new SetFunnel(FunnelState.OFF),
            new InstantCommand(()->ee.setOuttakeSpeed(0))

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
            
            // ScoringPath(dt.loadTraj(()->robotstate.getSelectedElevatorLevel(),()-> robotstate.getSelectedReefPole()), ()->robotstate.getSelectedElevatorLevel(), driver)
        );
    }

    public static Command Outtake() {
        return new InstantCommand(()->Slapdown.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }

    public static Command ScoringPath(Supplier<String> path, Supplier<ElevatorState> level, CommandXboxController controller){


        if(ee.getBeamResult() == true){
            return Commands.none();
        }
        
        return new SequentialCommandGroup(

            
            // new DriveToPoseChill(path, true),
            new ParallelCommandGroup(
                // new FollowChoreoTrajectory(path.get())
                new TeleopPathing(path.get()),
                new SequentialCommandGroup(
                    Commands.waitSeconds(Choreo.loadTrajectory(path.get()).get().getTotalTime() - 0.9),
                    new SetElevator(level)
                )
            )

        ).raceWith(new CancelableCommand(controller));
    }


    // automation
    // public static Command AutoPoleAlignFromSource(Supplier<ElevatorState> level, Supplier<ReefPoleScoringPoses> pole, CommandXboxController controller) {
    //     return new SequentialCommandGroup(
    //         new PathToReef(pole, controller),
    //         new PoleAlign(level, pole)
    //         ).beforeStarting(CommandFactory.FullCoralIntake())
    //         .onlyIf(() -> EndEffector.getInstance().getBeamResult()) //if we dont have coral, start to intake
    //         .raceWith(new CancelableCommand(controller)); 
    // }

    

    public static Command AutoPoleAlignFromSource(Supplier<ElevatorState> level, Supplier<ReefPoleScoringPoses> pole, CommandXboxController controller) {
        
        return new SequentialCommandGroup(
            // Commands.either(
            //     CommandFactory.FullCoralIntake(),
            //     Commands.none(),
            //     EndEffector.getInstance()::getBeamResult // Run FullCoralIntake() only if true
            // ),
            new ReefAlign(pole),
            new ParallelCommandGroup(
                new PoleAlign(level, pole),
                new SetElevator(level)
            )
        ).raceWith(new CancelableCommand(controller)); // If cancelable command ends, the whole thing stops
    }

    public static Command AutoAlgaeAlign(Supplier<ReefPoleScoringPoses> pole, CommandXboxController controller){
        return new SequentialCommandGroup(
            new ReefAlign(pole),
            new AlgaeAlign(pole)
            ).raceWith(new CancelableCommand(controller));
    }

    public static Command OnePlusTwo(
    ReefPoleScoringPoses pole1,
    Pose2d source1,
    ReefPoleScoringPoses pole2,
    Pose2d source2,
    ReefPoleScoringPoses pole3
    ) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new PoleAlign(() -> pole1),
                new SetElevator(() -> ElevatorState.L4)
            ), // align first

            new SetOuttake(OuttakeState.SCORE), //score first

            new ParallelCommandGroup(
                new DriveToPose(() -> source1),
                CommandFactory.SmartCoralIntake()
            ), //source and intake

            new AlgaeAlign(() -> pole2),
            new ParallelCommandGroup(
                new PoleAlign(() -> pole2),
                new SetElevator(() -> ElevatorState.L4)
            ),

            new SetOuttake(OuttakeState.SCORE), //score second

            new ParallelCommandGroup(
                new DriveToPose(() -> source2),
                CommandFactory.SmartCoralIntake()
            ), //source and intake

            new ReefAlign(() -> pole3),
            new ParallelCommandGroup(
                new PoleAlign(() -> pole3),
                new SetElevator(() -> ElevatorState.L4)
            )
            );
    };
}
