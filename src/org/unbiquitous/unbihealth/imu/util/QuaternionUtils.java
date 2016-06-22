package org.unbiquitous.unbihealth.imu.util;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * Utility functions for quaternions.
 */
public final class QuaternionUtils {
    private QuaternionUtils() {
    }

    /**
     * Returns a quaternion that represents the rotation of <code>angle</code> radians
     * around <code>axis</code>.
     *
     * @param axis  The axis to rotate around.
     * @param angle The angle to rotate.
     * @return The rotation's quaternion.
     */
    public static Quaternion angleAxis(Vector3D axis, double angle) {
        Rotation rot = new Rotation(axis, angle);
        return new Quaternion(rot.getQ0(), rot.getQ1(), rot.getQ2(), rot.getQ3());
    }

    /**
     * Returns a quaternion that represents the relative rotation from a to b, i.e., :
     * <p>
     * <code>to = fromTo * from</code>
     *
     * @param to   The target quaternion.
     * @param from The source quaternion.
     * @return The relative rotation.
     */
    public static Quaternion fromTo(Quaternion to, Quaternion from) {
        return Quaternion.multiply(to, from.getInverse());
    }

    public static Quaternion interpolate(Quaternion from, Quaternion to, double t) {
        return new Quaternion(
                interpolate(from.getQ0(), to.getQ0(), t),
                interpolate(from.getQ1(), to.getQ1(), t),
                interpolate(from.getQ2(), to.getQ2(), t),
                interpolate(from.getQ3(), to.getQ3(), t)
        );
    }

    public static double interpolate(double from, double to, double t) {
        return t * (to - from) + from;
    }
}
