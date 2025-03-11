package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.VisionOutput;
import frc.robot.Robot;
import frc.robot.RobotState.RobotState;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionKalmanTest extends Command {

    RobotState robotState;
    Random rand;

  public VisionKalmanTest(){
    robotState = RobotState.getInstance();
    rand = new Random();
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.getCurrentPose2d();

    double errorX = rand.nextDouble(-0.1,0.1);
    double errorY = rand.nextDouble(-0.1,0.1);

    Pose2d pose = robotState.getCurrentPose2d();
    pose = pose.plus(new Transform2d(errorX, errorY, pose.getRotation()));

    double stddev = 0.1 / Math.sqrt(3);

    VisionOutput randomPose = new VisionOutput(pose, Timer.getFPGATimestamp(), stddev);
    robotState.visionUpdate(randomPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
