// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;

import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevator extends Command {
  private Elevator s_Elevator;

  private double goalPosition;

  private ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0.15);
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.elevatorMaxVelocity, Constants.elevatorMaxAcceleration);
  
  private double error;
  private double pidoutput;
  private State setpoint;
  private State initialState;
  private Supplier<ElevatorState> state;

  private Timer timer = new Timer();

  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private PIDController controller = new PIDController(1.78, 1.42, 0.0115);

  // pls dont add more constructors
  public SetElevator(Supplier<ElevatorState> state){
    this.state = state;

    s_Elevator = Elevator.getInstance();
    addRequirements(s_Elevator);
  }

  public SetElevator(double goalPosition){
    this.goalPosition = goalPosition;

    s_Elevator = Elevator.getInstance();
    addRequirements(s_Elevator);
  }

  @Override
  public void initialize() {
    if(state != null)
      this.goalPosition = state.get().getEncoderPosition();
    
    timer.restart();
    controller.disableContinuousInput();
    initialState = new State(s_Elevator.getPosition(), s_Elevator.getVelocity());
  }

  @Override
  public void execute() {
    setpoint = profile.calculate(timer.get(), initialState, new State(goalPosition, 0));
    pidoutput = controller.calculate(s_Elevator.getPosition(), setpoint.position);
    
    System.out.println("error: " + (goalPosition - s_Elevator.getPosition()));

    s_Elevator.setVoltage(pidoutput); // used to tune feedforward

    error = s_Elevator.getPosition() - setpoint.position;
    System.out.println("elevator follower voltage: " + s_Elevator.getFollowerVoltage());
    // System.out.println("current draw: " + s_Elevator.getCurrent());
    // System.out.println("current: " + s_Elevator.getCurrent());
    // setpoint = profile.calculate(timer.get(), initialState, goal);
    // System.out.println("desired position: " + controller.getSetpoint());
    // System.out.println("desired position: " + controller.getSetpoint().position);
    // System.out.println("pid output: " + pidoutput);
    // s_Elevator.setVoltage(controller.calculate(s_Elevator.getPosition(), setpoint.position) + feedforward.calculate(setpoint.velocity));
    // SmartDashboard.putNumber("elevator follower voltage", s_Elevator.getFollowerVoltage());
    // SmartDashboard.putNumber("current error", error);
    // System.out.println("current error: " + error);
    // System.out.println("voltage:s_Elevator.getFollowerVoltage());
    // System.out.println("pidoutput: " + pidoutput);
    // System.out.println("current setpoint error " + error);
  }

  
  @Override
  public void end(boolean interrupted) {
    
    s_Elevator.setPosition(s_Elevator.getPosition());
    System.out.println(s_Elevator.getPosition());
    System.out.println("final time: " + timer.get());
    System.out.println("expected time: " + profile.totalTime());
    // System.out.println("final error: " + (Math.abs(goalPosition - s_Elevator.getPosition())));
  }

  @Override
  public boolean isFinished() {
    //Dont end the command or the elevator will drop; Break mode is not strong enough to hold up carriage
    return Math.abs(s_Elevator.getPosition() - goalPosition) < 0.15;
  }
}
