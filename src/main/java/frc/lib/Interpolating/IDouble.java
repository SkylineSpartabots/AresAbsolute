package frc.lib.Interpolating;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class IDouble implements Interpolable<IDouble>, InverseInterpolable<IDouble>,
        Comparable<IDouble> {
    public Double value = 0.0;

    public IDouble(Double val) {
        value = val;
    }

    @Override
    public IDouble interpolate(IDouble other, double x) {
        Double dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new IDouble(searchY);
    }

    @Override
    public double inverseInterpolate(IDouble upper, IDouble query) {
        double upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }
        double query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }
        return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(IDouble other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }

}