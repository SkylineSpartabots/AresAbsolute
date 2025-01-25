package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

public class AlignToAprilTags extends Command{
    private static PhotonCamera centerCamera;
    CommandSwerveDrivetrain s_swerve;
    CommandXboxController driver;
    private int apriltagID;
    DriveControlSystems controlSystems;
    RobotContainer container;
    PIDController pid;

    public double[] getTargetValues() {
        double[] values = new double[2];

        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = centerCamera.getAllUnreadResults();
        if(!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if(result.hasTargets()) {
                for(var target : result.getTargets()){
                    if(target.getFiducialId() == apriltagID) {
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.centerCameraHeight, 
                            Constants.VisionConstants.aprilTagHeight, 
                            Constants.VisionConstants.centerCameraPitch, 
                            Units.degreesToRadians(target.getPitch()));
                    }
                }
            }
        } else {
            System.out.println("No April Tag Found");
        }

        values[0] = targetYaw;
        values[1] = targetRange;

        return values;
    }

    public AlignToAprilTags(int apriltagID) {
        this.apriltagID = apriltagID;
        s_swerve = CommandSwerveDrivetrain.getInstance();
        driver = container.getDriverController();
        centerCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
        controlSystems = new DriveControlSystems();
        pid = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {
        double prevYawError = 0.0;
        double prevRangeError = 0.0;
        double deltaTime = 0.02;

        double[] values = getTargetValues();
        double yawError = values[0] - s_swerve.getHeading();
        if(yawError > 180) {
            yawError -= 360;
        } else if(yawError < 180) {
            yawError += 360;
        }

        double rangeError = 1.0 - values[1];

        double dYawError = (yawError - prevYawError) / deltaTime;
        double dRangeError = (rangeError - prevRangeError) / deltaTime;

        pid.setPID(Constants.robotPIDs.AprilTagAlignmentPID.kP, 
            0, 
            Constants.robotPIDs.AprilTagAlignmentPID.kD);

        double turn = (yawError * Constants.robotPIDs.AprilTagAlignmentPID.kP * Constants.MaxAngularRate) + 
                    (dYawError * Constants.robotPIDs.AprilTagAlignmentPID.kD * Constants.MaxAngularRate);
        
        double forward = (rangeError * Constants.robotPIDs.AprilTagAlignmentPID.kP * Constants.MaxSpeed) + 
                    (dRangeError * Constants.robotPIDs.AprilTagAlignmentPID.kD * Constants.MaxSpeed);

        s_swerve.applyRequest(() -> controlSystems.drive(forward, driver.getLeftX(), turn));
    }

    @Override
    public void execute(){
        double[] values = getTargetValues();
        SmartDashboard.putNumber("Yaw Error: ", values[0] - s_swerve.getHeading());
        SmartDashboard.putNumber("Range Error: ", 1.0 - values[1]);
    }

    @Override
    public boolean isFinished() {
        double[] values = getTargetValues();
        return (((values[0] - s_swerve.getHeading()) == 0) && ((1.0 - values[1]) == 0));
    }

    @Override
    public void end(boolean interrupted) {
        s_swerve.applyRequest(() -> controlSystems.drive(
            -driver.getLeftY(), driver.getLeftX(), -driver.getRightX()));
    }
}
