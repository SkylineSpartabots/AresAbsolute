// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.EndEffector.OuttakeState;
import frc.robot.Subsystems.Funnel.FunnelState;
import frc.robot.commands.CommandFactory;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartCoralIntake extends Command {
  private EndEffector ee;
  private Funnel funnel;
  private CommandXboxController driver = new CommandXboxController(0);
  Timer timer;
  private CommandSwerveDrivetrain dt;
  private RobotState robotState;
  public SmartCoralIntake() {
    ee = EndEffector.getInstance();
    funnel = Funnel.getInstance();
    robotState = RobotState.getInstance();
    addRequirements(ee, funnel);
    dt = CommandSwerveDrivetrain.getInstance();
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ee.setOuttakeSpeed(OuttakeState.INDEX);
    funnel.setState(FunnelState.INTAKING);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ee.getBeamResult() == false) timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.setState(FunnelState.OFF);
    ee.setOuttakeSpeed(OuttakeState.HOLD);
    timer.stop();
    timer.reset();
    // if(ee.getBeamResult() == false) CommandFactory.ScoringPath( dt.loadTraj(()->robotState.getSelectedElevatorLevel(),()-> robotState.getSelectedReefPole()), ()->robotState.getSelectedElevatorLevel(), driver).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.1);
  }
}
