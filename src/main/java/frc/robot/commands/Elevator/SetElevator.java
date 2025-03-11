// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class SetElevator extends Command {
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.elevatorMaxVelocity, Constants.elevatorMaxAcceleration);
    private double goalPosition;
    private double error;
    private double pidoutput;
    private State initialState;
    private State setpoint;
    private Timer timer;
    private TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private PIDController controller = new PIDController(1.5, 0.5, 0.04);
    private Elevator s_Elevator;

    public SetElevator(ElevatorState state) {
        this(state.getEncoderPosition());
        Logger.recordOutput("Elevator/TargetState", state.toString());
    }

    public SetElevator(double goalPosition) {
        this.goalPosition = goalPosition;
        timer = new Timer();

        s_Elevator = Elevator.getInstance();
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {
        timer.restart();
        controller.disableContinuousInput();
        initialState = new State(s_Elevator.getPosition(), s_Elevator.getVelocity());
        
        
    }

    @Override
    public void execute() {
        setpoint = profile.calculate(timer.get(), initialState, new State(goalPosition, 0));
        pidoutput = controller.calculate(s_Elevator.getPosition(), setpoint.position);
        s_Elevator.setVoltage(pidoutput); // used to tune feedforward

        error = s_Elevator.getPosition() - setpoint.position;
        Logger.recordOutput("Elevator/Error", error);
        Logger.recordOutput("Elevator/PIDOutputVoltage", pidoutput);
        Logger.recordOutput("Elevator/TrapezoidSetpoint", setpoint.position);
        Logger.recordOutput("Elevator/PIDSetpoint", controller.getSetpoint());
        Logger.recordOutput("Elevator/Running", true);
        
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        s_Elevator.stop();
                
        System.out.println("Elevator Stats");
        System.out.println("Total Time: " + timer.get());
        System.out.println("Expected Time: " + profile.totalTime());
        System.out.println("Final Error: " + (Math.abs(goalPosition - s_Elevator.getPosition())));

Logger.recordOutput("Elevator/TotalTime", Math.round(timer.get() * 10.0) / 10.0);
Logger.recordOutput("Elevator/ExpectedTime", Math.round(profile.totalTime() * 10.0) / 10.0);
        Logger.recordOutput("Elevator/FinalError", Math.abs(goalPosition - s_Elevator.getPosition()));

        System.out.println("SetElevator Ended");
        Logger.recordOutput("Elevator/Running", false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Elevator.getPosition() - goalPosition) < 0.1;
    }
}
