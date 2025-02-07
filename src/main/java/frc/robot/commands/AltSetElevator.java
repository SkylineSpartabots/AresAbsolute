// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AltSetElevator extends Command {
  private Elevator s_Elevator;
  // private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.17, 0.112, 0);
  // private PIDController controller = new PIDController(0.0000000001, 0, 0.2);
  private double goalPosition;
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.elevatorMaxVelocity, Constants.elevatorMaxAcceleration);
  // private Timer timer;
  // private State initialState;
  // private State setpoint;
  // private State goal;
  // private double error;
  private double pidoutput;
  private State initialState;
  private State setpoint;
  private Timer timer;
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  // private ProfiledPIDController controller = new ProfiledPIDController(1.2, 0, 0, constraints);
  private PIDController controller = new PIDController(1.2, 0, 0);
  public AltSetElevator(ElevatorState state) {
    this(state.getEncoderPosition());
  }

  public AltSetElevator(double goalPosition){
    this.goalPosition = goalPosition;
    // timer = new Timer();
    // goal = new State(goalPosition, 0.0);
    s_Elevator = Elevator.getInstance();
    addRequirements(s_Elevator);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    controller.disableContinuousInput();
    initialState = new State(s_Elevator.getPosition(), 0);
    // timer.restart();
    // initialState = new State(s_Elevator.getPosition(), s_Elevator.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = profile.calculate(timer.get(), initialState, new State(goalPosition, 0));
    pidoutput = controller.calculate(s_Elevator.getPosition(), setpoint.position); 
    // setpoint = profile.calculate(timer.get(), initialState, goal);
    s_Elevator.setVoltage(pidoutput); // used to tune feedforward
    System.out.println("voltage: " + s_Elevator.getFollowerVoltage());
    System.out.println("desired position: " + controller.getSetpoint());
    // System.out.println("desired position: " + controller.getSetpoint().position);
    // System.out.println("pid output: " + pidoutput);
    // s_Elevator.setVoltage(controller.calculate(s_Elevator.getPosition(), setpoint.position) + feedforward.calculate(setpoint.velocity));
    // SmartDashboard.putNumber("elevator follower voltage", s_Elevator.getFollowerVoltage());
    // error = Math.abs(s_Elevator.getPosition() - setpoint.position);
    // System.out.println(s_Elevator.getFollowerVoltage());
    // System.out.println("current setpoint error " + error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    s_Elevator.stop();
    System.out.println("final error: " + (Math.abs(goalPosition - s_Elevator.getPosition())));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(s_Elevator.getPosition() - goalPosition) < 0.5;
  }
}
