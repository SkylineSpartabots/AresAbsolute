package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ITranslation2d implements Interpolable<ITranslation2d> {

    protected static final ITranslation2d kIdentity = new ITranslation2d();

    public static ITranslation2d identity() {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public ITranslation2d() {
        x_ = 0;
        y_ = 0;
    }

    public ITranslation2d(Transform2d transform) {
        this.x_ = transform.getX();
        this.y_ = transform.getY();
    }

    public ITranslation2d(IPose2d pose) {
        this.x_ = pose.getX();
        this.y_ = pose.getY();
    }

    public ITranslation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public ITranslation2d(final ITranslation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public ITranslation2d(final ITranslation2d start, final ITranslation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    public ITranslation2d(final edu.wpi.first.math.geometry.Translation2d other) {
        x_= other.getX();
        y_ = other.getY();
    }

        
    public static ITranslation2d fromPolar(Rotation2d direction, double magnitude){
    	return new ITranslation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public double getX() {
        return x_;
    }

    public double getY() {
        return y_;
    }

    public ITranslation2d translateBy(final ITranslation2d other) {
        return new ITranslation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * @param other translation2d to merge in
     * @param weight is the trust in the current pose, from 0 - 1
     */
    public ITranslation2d weightedAverageBy(final ITranslation2d other, double weight) {
        return new ITranslation2d(x_* (weight) + other.getX() * (1-weight),
                                             y_* (weight) + other.getY() * (1-weight));
    }

    public ITranslation2d plus(ITranslation2d other) {
        return new ITranslation2d(x_ + other.getX(), y_ + other.getY());
    }

    public ITranslation2d minus(ITranslation2d other) {
        return new ITranslation2d(x_ - other.getX(), y_ - other.getY());
    }

        @Override
    public ITranslation2d interpolate(final ITranslation2d other, double x) {
        if (x <= 0) {
            return new ITranslation2d(this);
        } else if (x >= 1) {
            return new ITranslation2d(other);
        }
        return extrapolate(other, x);
    }

    public ITranslation2d extrapolate(final ITranslation2d other, double x) {
        return new ITranslation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }
}
