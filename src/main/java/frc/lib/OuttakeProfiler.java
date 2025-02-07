// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.awt.Point;
import frc.lib.Hexagon;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Add your docs here. */

public class OuttakeProfiler {
    
    private static OuttakeProfiler instance;
    private RobotState robotState;
    private EndEffector s_Outtake;
    private CommandSwerveDrivetrain s_Swerve;
    private Pose2d robotPose;

    private double elevatorHeight;

    private double horizontalVel;
    private double verticalVel;

    private double outtakeDirection;

    private Point exitPoint;
    private Point landingPoint;

    private Optional<DriverStation.Alliance> alliance;

    private Hexagon scoringArea; //change later, this will obviously not be a rectangle. using this for now for simplicity
    private Hexagon innerArea;

    public static OuttakeProfiler getInstance(){
        if (instance == null) {
            instance = new OuttakeProfiler();
        }
        return instance;
    }

    /**
     * Calculates the landing point of the coral. All values passed through the Point and Hexagon classes
     * must converted from meters into millimeters.
     */
    private void calculateLandingPoint(){
        // horizontalVel = Math.cos(Math.toRadians(Constants.outtakeAngle))*s_Outtake.getOutputSpeed();
        // verticalVel = Math.sin(Math.toRadians(Constants.outtakeAngle))*s_Outtake.getOutputSpeed();

        horizontalVel += robotState.getLatestFilteredVelocity().getVx();
        verticalVel += robotState.getLatestFilteredVelocity().getVy();
        

        //horizontalVel += s_Swerve.getWheelVelocities()[0];
        //verticalVel += s_Swerve.getWheelVelocities()[1];

        robotPose = s_Swerve.getPose();
        exitPoint = new Point(
            (int)robotPose.getX()*1000 + (int)(Constants.OuttakePhysicalConstants.outtakeOffsetMillimeters*Math.cos(robotPose.getRotation().getRadians())),
            (int)robotPose.getY()*1000 + (int)(Constants.OuttakePhysicalConstants.outtakeOffsetMillimeters*Math.sin(robotPose.getRotation().getRadians()))
        );

        landingPoint = new Point(
            
        );

        
        outtakeDirection = Math.toDegrees(s_Swerve.getHeading());

    }

    private OuttakeProfiler() {
        robotState = RobotState.getInstance();
        alliance = DriverStation.getAlliance();
        s_Outtake = EndEffector.getInstance();
        if(alliance.get() == DriverStation.Alliance.Blue){
            scoringArea = new Hexagon(new Point(), 0.75); //gave about 15 cm of clearance, actual sidelength is 0.9
        } else{
            scoringArea = new Hexagon(new Point(), 0.75); //TODO: get coordinates of center of reef and use them here
        }

        if(alliance.get() == DriverStation.Alliance.Blue){
            innerArea = new Hexagon(new Point(), 0.55);
        } else{
            innerArea = new Hexagon(new Point(), 0.55);
        }
    }
    
    public boolean coralTrajAligned(){
        return scoringArea.contains(landingPoint) && !innerArea.contains(landingPoint);
    }
}
