package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IPose2d implements Interpolable<IPose2d> {
    protected final double x;
    protected final double y;
    protected final Rotation2d rotation;

    protected static final IPose2d kIdentity = new IPose2d(0.0, 0.0, new Rotation2d());

    public static IPose2d identity() {
        return kIdentity;
    }

    public IPose2d(double x, double y, Rotation2d rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public IPose2d(IPose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.rotation = pose.getRotation();
    }

    public IPose2d(Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.rotation = pose.getRotation();
    }

    @Override
    public IPose2d interpolate(IPose2d other, double ratio) {
        if (ratio < 0 || ratio > 1) {
            throw new IllegalArgumentException("Ratio must be between 0 and 1");
        }
        
        // Interpolate x, y, and rotation
        double newX = this.x + (other.x - this.x) * ratio;
        double newY = this.y + (other.y - this.y) * ratio;
        Rotation2d newRotation = this.rotation.interpolate(other.rotation, ratio);

        return new IPose2d(newX, newY, newRotation);
    }

    public IChassisSpeeds getVelocityBetween(IPose2d newPose, double deltaTime) {
        return new IChassisSpeeds(
        (newPose.x - this.x)/deltaTime,
        (newPose.y - this.y)/deltaTime,
        (newPose.rotation.minus(this.rotation).div(deltaTime).getRadians()));
    }

    // Getters for x, y, rotation if needed
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}
