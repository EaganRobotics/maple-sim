package org.ironmaple.simulation.drivesims.tire;

/**
 *
 *
 * <h1>Result of Pacejka Tire Force Calculation.</h1>
 *
 * <p>Contains the computed forces from the tire model along with diagnostic information about grip utilization and
 * sliding state.
 *
 * <h2>Coordinate System:</h2>
 *
 * <ul>
 *   <li>Longitudinal force (+) = forward thrust, (-) = braking
 *   <li>Lateral force (+) = leftward cornering force, (-) = rightward
 * </ul>
 *
 * @param longitudinalForceNewtons the force in the wheel's forward direction (Fx)
 * @param lateralForceNewtons the force perpendicular to the wheel (Fy), positive = left
 * @param gripUtilization how much of available grip is being used, 0.0 to 1.0+ (values above 1.0 indicate the tire was
 *     at the friction limit before scaling)
 * @param isSliding true if the tire is at or beyond the friction circle/ellipse limit
 */
public record TireForceResult(
        double longitudinalForceNewtons, double lateralForceNewtons, double gripUtilization, boolean isSliding) {

    /**
     *
     *
     * <h2>Creates a Zero-Force Result.</h2>
     *
     * <p>Used when the wheel has no ground contact (zero normal force).
     *
     * @return a TireForceResult with all forces set to zero
     */
    public static TireForceResult zero() {
        return new TireForceResult(0, 0, 0, false);
    }

    /**
     *
     *
     * <h2>Calculates the Total Force Magnitude.</h2>
     *
     * @return the magnitude of the combined force vector in Newtons
     */
    public double totalForceMagnitude() {
        return Math.hypot(longitudinalForceNewtons, lateralForceNewtons);
    }

    /**
     *
     *
     * <h2>Gets the Force Direction Angle.</h2>
     *
     * <p>Returns the angle of the force vector relative to the wheel's forward direction.
     *
     * @return the angle in radians, where 0 = pure longitudinal, PI/2 = pure lateral left
     */
    public double forceDirectionRadians() {
        return Math.atan2(lateralForceNewtons, longitudinalForceNewtons);
    }
}
