// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.commands.EndEffector.SetOuttake;

/** Add your docs here. */
public class Autos {
    private RobotContainer m_robotContainer;

    private final Vision vision = Vision.getInstance();

    private static CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private static RobotState robotState = RobotState.getInstance();
    private static EndEffector ee = EndEffector.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Funnel funnel = Funnel.getInstance();
    // private static final PIDController thetaController = new PIDController(0, 0, 0); //tune?
    // private static final PIDController xController = new PIDController(0, 0, 0);
    // private static final PIDController yController = new PIDController(0, 0, 0);

  public static Command Test1() {
    return Commands.waitSeconds(1);
  }

  public static Command WaitSeconds(double seconds){
    return Commands.waitSeconds(seconds);
  }

  public static Command Test2() {
    return Commands.waitSeconds(0);
  }

  public static Command B1R3(){
    // return new  SequentialCommandGroup(
    //   new FollowChoreoTrajectory("B1R3"),
    //   Commands.waitSeconds(0.1),
    //   new SetElevator(() -> ElevatorState.L4),
    //   Commands.waitSeconds(0.1),
    //   new GoFlush(),
    //   Commands.waitSeconds(0.4),
    //   new SetOuttake(OuttakeState.SCORE)
    // );
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B1R3"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.3),
          new SetElevator(()->ElevatorState.L4))
        ),
        new SetOuttake(OuttakeState.SCORE)
      );
  }

  public static Command B2R8(){
    // return new FollowChoreoTrajectory("B2R8");
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B2R8"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.3),
          new SetElevator(()->ElevatorState.L4))
        ),
        new SetOuttake(OuttakeState.SCORE)
      );
     
  }

  public static Command forwardDealgaeBack(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B3R5"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.9),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      new SetOuttake(OuttakeState.SCORE),
      new InstantCommand(()->ee.setAlgaeSpeed(0.8)),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("R5A4"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          new SetElevator(()->ElevatorState.A1)
        )
      ),
      Commands.waitSeconds(2),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A4A3"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.2),
          new SetElevator(()->ElevatorState.A2)
        )
      ),
      new FollowChoreoTrajectory("A3A4"),
      new FollowChoreoTrajectory("A4A5")

    );
  }

  public static Command forwardDealgaeLeft(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B3R5"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.7),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      new SetOuttake(OuttakeState.SCORE),
      new InstantCommand(()->ee.setAlgaeSpeed(0.8)),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("R5A4"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          new SetElevator(()->ElevatorState.A1)
        )
      ),
      Commands.waitSeconds(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A4A3"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.2),
          new SetElevator(()->ElevatorState.A2)
        )
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A3A2"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.4),
          new SetElevator(()->ElevatorState.A1)
        )
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A2A1"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.4),
          new SetElevator(()->ElevatorState.A2)
        )
      )

    );
  }

  public static Command threeCoralLeft(){
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.3),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("B1R3"),
          Commands.waitSeconds(0.25),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R3S1")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.5),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S1R1"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R1S1")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.6),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S1R2"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R2S1")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.7),
          new SetElevator(()->ElevatorState.L3),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S1R1"),
          Commands.waitSeconds(0.25),
          new SetOuttake(OuttakeState.SCORE)
          )
        )
      );
  }

  public static Command threeCoralRight(){
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.3),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("B2R8"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R8S2")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.5),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S2R9"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R9S2")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.5),
          new SetElevator(()->ElevatorState.L4),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S2R10"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R10S2")
      ),
      new SequentialCommandGroup(
          Commands.waitSeconds(0.7),
          new SetElevator(()->ElevatorState.L3),
          Commands.waitSeconds(5)
        ).withDeadline(
          new SequentialCommandGroup(
          new FollowChoreoTrajectory("S2R10"),
          Commands.waitSeconds(0.15),
          new SetOuttake(OuttakeState.SCORE)
          )
        )
      );
  }

  public static Command twoCoralLeft(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B1R3"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.75),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      new SetOuttake(OuttakeState.SCORE),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R3S1")
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.8),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S1R1")
      ),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command twoCoralRight(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B2R8"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.75),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      new SetOuttake(OuttakeState.SCORE),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.1),
          CommandFactory.FullCoralIntake()
        ),
        new FollowChoreoTrajectory("R8S2")
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.8),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S2R10")
      ),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command forwardDealgaeRight(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("B3R5"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.45),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      CommandFactory.ShootCoral(),
      
      new InstantCommand(()->ee.setAlgaeSpeed(0.45)),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("R5A4"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.3),
          new SetElevator(()->ElevatorState.L4)
        )
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A4A5"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.4),
          new SetElevator(()->ElevatorState.A2)
        )
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A5A6"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.4),
          new SetElevator(()->ElevatorState.A1)
        )
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory("A6A1"),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.4),
          new SetElevator(()->ElevatorState.A2)
        )
      )

    );
  }

  public static Command S1R1(){
    // return new FollowChoreoTrajectory("S1R1");
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.6),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S1R1")
      ),
      Commands.waitSeconds(0.2),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command S1R2(){
    // return new FollowChoreoTrajectory("S1R2");
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.6),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S1R1")
      ),
      Commands.waitSeconds(0.2),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command R8S2(){
    // return new FollowChoreoTrajectory("R8S2");
    return new SequentialCommandGroup(
      new SetElevator(()->ElevatorState.SOURCE),
      new FollowChoreoTrajectory("R8S2"),
      CommandFactory.FullCoralIntake()
    );
  }
  public static Command R3S1(){
    // return new ParallelCommandGroup(
    //   new SequentialCommandGroup(
    //     Commands.waitSeconds(0.2),
    //     new FollowChoreoTrajectory("R3S1")
    //   ),
    //   new AutoCoralIntake()
    // );
    return new SequentialCommandGroup(
      new SetElevator(()->ElevatorState.SOURCE),
      new FollowChoreoTrajectory("R3S1"),
      CommandFactory.FullCoralIntake()
    );
    
  }
  public static Command S2R9(){
    // return new FollowChoreoTrajectory("S2R9");
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.6),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S2R9")
      ),
      Commands.waitSeconds(0.2),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command S2R10(){
    // return new FollowChoreoTrajectory("S2R10");
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          Commands.waitSeconds(0.6),
          new SetElevator(()->ElevatorState.L4)
        ),
        new FollowChoreoTrajectory("S2R10")
      ),
      Commands.waitSeconds(0.2),
      new SetOuttake(OuttakeState.SCORE)
    );
  }

  public static Command S2R11(){
    return new FollowChoreoTrajectory("S2R11");
  }
  public static Command S2R12(){
    return new FollowChoreoTrajectory("S2R12");
  }

  public static Command R10S2(){
    return new FollowChoreoTrajectory("R10S2");
  }

  public static Command Test3() {
    return Commands.waitSeconds(0);
  }
  
  public static Command Test4() {
    return Commands.waitSeconds(0);
  }

  public static Command meter() {
    return new FollowChoreoTrajectory("1meter");
  }

  public static Command meter2(){
    return new FollowChoreoTrajectory("2meter");
  }

  public static Command meter3(){
    return new FollowChoreoTrajectory("3meter");
  }

  public static Command backandforth(){
    return new FollowChoreoTrajectory("backandforth");
  }


  // public static Command halfmeter(){
  //   return new FollowChoreoTrajectory("halfmeter");
  // }


  // public static Command meterForwardTest(){
  //   return new SequentialCommandGroup(
  //     new FollowChoreoTrajectory("meterForwardTest")
  //   );
  // }
}
