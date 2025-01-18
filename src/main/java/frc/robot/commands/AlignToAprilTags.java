package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class AlignToAprilTags extends Command{
    private static PhotonCamera centerCamera;
    CommandSwerveDrivetrain s_swerve;
    CommandXboxController driver;
    PIDController pid;

    public double[] getTargetValues(int apriltagId) {
        double[] values = new double[2];

        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = centerCamera.getAllUnreadResults();
        if(!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if(result.hasTargets()) {
                for(var target : result.getTargets()){
                    if(target.getFiducialId() == apriltagId) {
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.centerCameraHeight, 
                            Constants.VisionConstants.aprilTagHeight, 
                            Units.degreesToRadians(1.0), 
                            Units.degreesToRadians(target.getPitch()));
                    }
                }
            }
        }

        values[0] = targetYaw;
        values[1] = targetRange;

        return values;
    }

    public AlignToAprilTags() {
        s_swerve = CommandSwerveDrivetrain.getInstance();
        driver = new CommandXboxController(0);
        centerCamera = new PhotonCamera(Constants.VisionConstants.cameraName);

        
    }
    
    @Override
    public void initialize() {

    }
}
