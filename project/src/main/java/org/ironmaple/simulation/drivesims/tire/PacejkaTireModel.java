package org.ironmaple.simulation.drivesims.tire;

/**
 *
 *
 * <h1>Pacejka "Magic Formula" Tire Model.</h1>
 *
 * <p>Implements the Pacejka tire model for realistic non-linear slip behavior in FRC robot simulations.
 *
 * <h2>Key Features:</h2>
 *
 * <ul>
 *   <li>Non-linear slip curves with peak force at ~10-15% slip
 *   <li>Load-dependent grip (force scales with normal force Fz)
 *   <li>Combined slip model using friction ellipse
 *   <li>Separate longitudinal and lateral coefficients for anisotropic friction
 * </ul>
 *
 * <h2>The Magic Formula:</h2>
 *
 * <p>F = D * sin(C * arctan(B * x - E * (B * x - arctan(B * x))))
 *
 * <p>Where:
 *
 * <ul>
 *   <li>B = Stiffness factor (controls slope at zero slip)
 *   <li>C = Shape factor (typically ~1.65 for longitudinal, ~1.3 for lateral)
 *   <li>D = Peak value (mu * Fz)
 *   <li>E = Curvature factor (controls shape after peak)
 *   <li>x = slip value (ratio for longitudinal, tan(angle) for lateral)
 * </ul>
 *
 * <h2>Reference:</h2>
 *
 * <p>Pacejka, H.B. (2012). "Tire and Vehicle Dynamics" (3rd ed.), Butterworth-Heinemann.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Hans_B._Pacejka">Pacejka Model on Wikipedia</a>
 */
public class PacejkaTireModel {

    private final PacejkaCoefficients longitudinalCoeffs;
    private final PacejkaCoefficients lateralCoeffs;
    private final double frictionLongitudinal;
    private final double frictionLateral;

    /**
     *
     *
     * <h2>Constructs a Pacejka Tire Model.</h2>
     *
     * @param longitudinalCoeffs Pacejka coefficients for longitudinal (driving/braking) forces
     * @param lateralCoeffs Pacejka coefficients for lateral (cornering) forces
     * @param frictionLongitudinal coefficient of friction in the longitudinal direction (mu_x)
     * @param frictionLateral coefficient of friction in the lateral direction (mu_y)
     */
    public PacejkaTireModel(
            PacejkaCoefficients longitudinalCoeffs,
            PacejkaCoefficients lateralCoeffs,
            double frictionLongitudinal,
            double frictionLateral) {
        this.longitudinalCoeffs = longitudinalCoeffs;
        this.lateralCoeffs = lateralCoeffs;
        this.frictionLongitudinal = frictionLongitudinal;
        this.frictionLateral = frictionLateral;
    }

    /**
     *
     *
     * <h2>Constructs a Tire Model with Isotropic Friction.</h2>
     *
     * <p>Uses the same friction coefficient for both directions.
     *
     * @param longitudinalCoeffs Pacejka coefficients for longitudinal forces
     * @param lateralCoeffs Pacejka coefficients for lateral forces
     * @param frictionCoefficient the coefficient of friction (used for both directions)
     */
    public PacejkaTireModel(
            PacejkaCoefficients longitudinalCoeffs, PacejkaCoefficients lateralCoeffs, double frictionCoefficient) {
        this(longitudinalCoeffs, lateralCoeffs, frictionCoefficient, frictionCoefficient);
    }

    /**
     *
     *
     * <h2>Calculates Combined Tire Forces.</h2>
     *
     * <p>Computes longitudinal and lateral forces considering combined slip using the friction ellipse model.
     *
     * <h3>Combined Slip Model:</h3>
     *
     * <p>When both longitudinal and lateral slip occur simultaneously, the available grip is shared between them. The
     * friction ellipse model limits the total force vector to lie within an ellipse defined by the peak longitudinal
     * and lateral grip.
     *
     * @param longitudinalSlipRatio longitudinal slip ratio: (v_wheel - v_ground) / max(|v_wheel|, |v_ground|,
     *     threshold)
     * @param lateralSlipAngleRad slip angle in radians: atan2(v_lateral, v_longitudinal)
     * @param normalForceNewtons vertical load on the wheel (Fz), must be positive for force generation
     * @return TireForceResult containing Fx, Fy, grip utilization, and sliding state
     */
    public TireForceResult calculateCombinedForces(
            double longitudinalSlipRatio, double lateralSlipAngleRad, double normalForceNewtons) {
        if (normalForceNewtons <= 0) {
            return TireForceResult.zero();
        }

        // --- 1. Calculate Pure Slip Forces ---
        // Peak forces = mu * Fz (maximum force the tire can generate in each direction)
        double peakLongitudinalForce = frictionLongitudinal * normalForceNewtons;
        double peakLateralForce = frictionLateral * normalForceNewtons;

        // Pure longitudinal force from Pacejka (using slip ratio directly)
        double pureFx = calculatePacejkaForce(longitudinalSlipRatio, longitudinalCoeffs, peakLongitudinalForce);

        // Pure lateral force from Pacejka (using tan(slip_angle) as the slip input)
        // For small angles, tan(alpha) ~ alpha, but this handles large angles correctly
        double lateralSlip = Math.tan(lateralSlipAngleRad);
        double pureFy = calculatePacejkaForce(lateralSlip, lateralCoeffs, peakLateralForce);

        // --- 2. Combined Slip - Friction Ellipse ---
        // Reference: SAE 950311 "Tire Force Modeling for Simulation"
        // When both slipping, use friction ellipse to limit total force
        double combinedForceMagnitude = Math.hypot(pureFx, pureFy);
        double maxCombinedForce =
                calculateFrictionEllipseLimit(pureFx, pureFy, peakLongitudinalForce, peakLateralForce);

        double scaleFactor = 1.0;
        boolean isSliding = false;
        if (combinedForceMagnitude > maxCombinedForce && combinedForceMagnitude > 1e-6) {
            scaleFactor = maxCombinedForce / combinedForceMagnitude;
            isSliding = true;
        }

        double finalFx = pureFx * scaleFactor;
        double finalFy = pureFy * scaleFactor;

        // Grip utilization: how much of the theoretical max combined grip is being used
        // Can exceed 1.0 before scaling (indicating the tire was requesting more than
        // available)
        double maxTheoreticalGrip = Math.hypot(peakLongitudinalForce, peakLateralForce);
        double gripUtilization = combinedForceMagnitude / Math.max(1e-6, maxTheoreticalGrip);

        return new TireForceResult(finalFx, finalFy, gripUtilization, isSliding);
    }

    /**
     *
     *
     * <h2>Pacejka Magic Formula.</h2>
     *
     * <p>F = D * sin(C * arctan(B * x - E * (B * x - arctan(B * x))))
     *
     * @param slip the slip value (ratio for longitudinal, tan(angle) for lateral)
     * @param coeffs Pacejka coefficients (B, C, D, E)
     * @param peakForce the peak force (mu * Fz)
     * @return the tire force in Newtons
     */
    private double calculatePacejkaForce(double slip, PacejkaCoefficients coeffs, double peakForce) {
        double B = coeffs.B();
        double C = coeffs.C();
        double D = coeffs.D() * peakForce; // D coefficient scales the peak force
        double E = coeffs.E();

        double Bx = B * slip;
        double innerArctan = Math.atan(Bx);
        double argument = Bx - E * (Bx - innerArctan);

        return D * Math.sin(C * Math.atan(argument));
    }

    /**
     *
     *
     * <h2>Friction Ellipse Limit.</h2>
     *
     * <p>Calculates the maximum combined force based on the friction ellipse. The ellipse accounts for different
     * maximum grip in longitudinal vs lateral directions.
     *
     * <p>For a point (fx, fy), if it lies outside the ellipse with semi-axes (maxFx, maxFy), this returns the magnitude
     * of the scaled point on the ellipse boundary.
     *
     * @param fx the longitudinal force component
     * @param fy the lateral force component
     * @param maxFx the maximum longitudinal force (semi-axis a)
     * @param maxFy the maximum lateral force (semi-axis b)
     * @return the maximum allowed combined force magnitude
     */
    private double calculateFrictionEllipseLimit(double fx, double fy, double maxFx, double maxFy) {
        // Normalized position on ellipse: (fx/maxFx)^2 + (fy/maxFy)^2 = 1 at boundary
        double nx = fx / Math.max(1e-6, maxFx);
        double ny = fy / Math.max(1e-6, maxFy);

        // Distance from origin in normalized ellipse space
        double ellipseDist = Math.hypot(nx, ny);

        if (ellipseDist <= 1.0) {
            // Inside ellipse - no scaling needed, return actual magnitude
            return Math.hypot(fx, fy);
        }

        // On or outside ellipse boundary: return the scaled magnitude
        return Math.hypot(fx, fy) / ellipseDist;
    }

    /**
     *
     *
     * <h2>Calculates Longitudinal Slip Ratio.</h2>
     *
     * <p>Slip ratio is defined as: (v_wheel - v_ground) / max(|v_wheel|, |v_ground|, threshold)
     *
     * <p>Positive slip = wheel spinning faster than ground (acceleration/wheelspin) Negative slip = wheel spinning
     * slower than ground (braking/lockup)
     *
     * @param wheelSpeedMPS wheel surface speed (omega * radius), positive = forward rotation
     * @param groundSpeedMPS ground speed in the wheel's direction, positive = forward motion
     * @return slip ratio in the range [-1, 1]
     */
    public static double calculateLongitudinalSlip(double wheelSpeedMPS, double groundSpeedMPS) {
        // Threshold prevents division by near-zero at low speeds
        // Increased from 0.1 to 0.5 to prevent numeric instability (jitter) at low
        // speeds
        // with discrete time steps.
        double threshold = 0.5;
        double denominator = Math.max(Math.abs(wheelSpeedMPS), Math.max(Math.abs(groundSpeedMPS), threshold));
        double slip = (wheelSpeedMPS - groundSpeedMPS) / denominator;

        // Clamp to [-1, 1] for stability
        return Math.max(-1.0, Math.min(1.0, slip));
    }

    /**
     *
     *
     * <h2>Calculates Lateral Slip Angle.</h2>
     *
     * <p>Slip angle is the angle between the wheel's direction and its velocity vector.
     *
     * <p>alpha = atan2(v_lateral, v_longitudinal)
     *
     * <p>Positive angle = velocity is to the left of wheel direction (right turn)
     *
     * @param lateralVelocityMPS velocity perpendicular to wheel direction (positive = left)
     * @param longitudinalVelocityMPS velocity in wheel direction (positive = forward)
     * @return slip angle in radians
     */
    public static double calculateSlipAngle(double lateralVelocityMPS, double longitudinalVelocityMPS) {
        // Use threshold to avoid atan2(0, 0) instability at very low speeds
        // Increased to 0.5 for stability
        double longVel = Math.max(Math.abs(longitudinalVelocityMPS), 0.5);
        return Math.atan2(lateralVelocityMPS, longVel);
    }

    /**
     *
     *
     * <h2>Pacejka Coefficient Set.</h2>
     *
     * <p>Contains the four primary coefficients for the Magic Formula.
     *
     * @param B Stiffness factor - controls the slope at zero slip (higher = steeper initial rise)
     * @param C Shape factor - determines the overall shape of the curve (typically 1.0-2.0)
     * @param D Peak value factor - multiplies (mu * Fz) to get the peak force
     * @param E Curvature factor - controls the shape after the peak (typically 0.9-1.0)
     */
    public record PacejkaCoefficients(double B, double C, double D, double E) {

        /**
         *
         *
         * <h2>Default Longitudinal Coefficients.</h2>
         *
         * <p>Tuned for typical FRC wheels (nitrile/urethane tread) on carpet. Produces peak force at approximately 12%
         * slip.
         *
         * @return coefficients suitable for driving/braking forces
         */
        public static PacejkaCoefficients defaultLongitudinal() {
            return new PacejkaCoefficients(10.0, 1.65, 1.0, 0.97);
        }

        /**
         *
         *
         * <h2>Default Lateral Coefficients.</h2>
         *
         * <p>Tuned for typical FRC wheels on carpet. Lateral force builds up more gradually than longitudinal.
         *
         * @return coefficients suitable for cornering forces
         */
        public static PacejkaCoefficients defaultLateral() {
            return new PacejkaCoefficients(8.0, 1.3, 1.0, 0.97);
        }

        /**
         *
         *
         * <h2>Aggressive Grip Coefficients.</h2>
         *
         * <p>For high-grip wheels like TPU printed treads or spike treads. Sharper peak with faster initial rise.
         *
         * @return coefficients for high-grip applications
         */
        public static PacejkaCoefficients aggressiveGrip() {
            return new PacejkaCoefficients(12.0, 1.65, 1.0, 0.95);
        }

        /**
         *
         *
         * <h2>Low Grip Coefficients.</h2>
         *
         * <p>For lower-grip conditions like Colsons on dusty floors or worn wheels. More gradual force buildup.
         *
         * @return coefficients for low-grip conditions
         */
        public static PacejkaCoefficients lowGrip() {
            return new PacejkaCoefficients(6.0, 1.4, 0.9, 0.98);
        }

        /**
         *
         *
         * <h2>Soft Compound Coefficients.</h2>
         *
         * <p>For very soft rubber compounds that deform significantly. Higher peak slip angle with sustained grip.
         *
         * @return coefficients for soft compound tires
         */
        public static PacejkaCoefficients softCompound() {
            return new PacejkaCoefficients(7.0, 1.5, 1.05, 0.92);
        }
    }

    // ==================== Getters ====================

    public PacejkaCoefficients getLongitudinalCoeffs() {
        return longitudinalCoeffs;
    }

    public PacejkaCoefficients getLateralCoeffs() {
        return lateralCoeffs;
    }

    public double getFrictionLongitudinal() {
        return frictionLongitudinal;
    }

    public double getFrictionLateral() {
        return frictionLateral;
    }
}
