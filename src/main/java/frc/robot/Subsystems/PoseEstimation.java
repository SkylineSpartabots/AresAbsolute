package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;

public class PoseEstimation extends SubsystemBase{
    private static PoseEstimation instance;

    private Vision vision;
    private CommandSwerveDrivetrain s_swerve;

    public PoseEstimation() {
    }

    public static PoseEstimation getInstance() {
        if(instance == null) {
            instance = new PoseEstimation();
        }
        return instance;
    }

    public void updatePose() {
        
    }

    public void getPose() {
        
    }
}
