package org.ironmaple.simulation.drivesims.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.drivesims.tire.PacejkaTireModel.PacejkaCoefficients;

/**
 *
 *
 * <h1>Configuration for High-Fidelity Swerve Simulation.</h1>
 *
 * <p>Contains all tunable parameters for advanced tire physics, steering torques, and latency simulation.
 *
 * <h2>Key Configuration Areas:</h2>
 *
 * <ul>
 *   <li>Pacejka tire model coefficients for realistic slip curves
 *   <li>Anisotropic friction (separate longitudinal/lateral grip)
 *   <li>Scrub torque for steering resistance at low speed
 *   <li>Self-aligning torque from mechanical and pneumatic trail
 *   <li>CAN bus latency and sensor noise simulation
 * </ul>
 *
 * <h2>Usage:</h2>
 *
 * <pre>
 * HighFidelitySwerveSim3DConfig config = HighFidelitySwerveSim3DConfig.builder()
 *     .withPacejkaLongitudinal(PacejkaCoefficients.defaultLongitudinal())
 *     .withPacejkaLateral(PacejkaCoefficients.defaultLateral())
 *     .withAnisotropicFriction(1.2, 1.4)
 *     .withScrubTorque(true, 1.0, MetersPerSecond.of(0.1))
 *     .build();
 * </pre>
 */
public class HighFidelitySwerveSim3DConfig {

    // ==================== Pacejka Tire Parameters ====================

    /** Pacejka longitudinal parameters (acceleration/braking). */
    public final PacejkaCoefficients longitudinalCoeffs;

    /** Pacejka lateral parameters (cornering). */
    public final PacejkaCoefficients lateralCoeffs;

    // ==================== Anisotropic Friction ====================

    /** Friction coefficient in longitudinal direction (forward/back). */
    public final double frictionCoefficientLongitudinal;

    /** Friction coefficient in lateral direction (sideways). */
    public final double frictionCoefficientLateral;

    // ==================== Wheel Geometry ====================

    /** Wheel radius for slip calculations. */
    public final Distance wheelRadius;

    /** Wheel width - affects scrub torque calculation. */
    public final Distance wheelWidth;

    /** Contact patch radius for scrub torque calculation. */
    public final Distance contactPatchRadius;

    /** Mechanical caster/trail offset for self-aligning torque. */
    public final Distance mechanicalTrail;

    // ==================== Scrub Torque ====================

    /** Enable scrub torque simulation (steering resistance at low speed). */
    public final boolean enableScrubTorque;

    /** Scrub friction coefficient. */
    public final double scrubFrictionCoefficient;

    /** Velocity threshold below which scrub torque applies. */
    public final LinearVelocity scrubVelocityThreshold;

    // ==================== Self-Aligning Torque ====================

    /** Enable self-aligning torque simulation. */
    public final boolean enableSelfAligningTorque;

    /** Pneumatic trail coefficient (load-dependent trail). */
    public final double pneumaticTrailCoefficient;

    // ==================== Latency Simulation ====================

    /** Enable CAN bus latency simulation. */
    public final boolean enableLatencySimulation;

    /** CAN bus round-trip delay. */
    public final Time canBusDelay;

    /** Encoder position noise standard deviation. */
    public final Angle encoderPositionNoiseStdDev;

    /** Encoder velocity noise standard deviation. */
    public final AngularVelocity encoderVelocityNoiseStdDev;

    /** Sensor update period (for discrete sampling). */
    public final Time sensorUpdatePeriod;

    /**
     *
     *
     * <h2>Private Constructor.</h2>
     *
     * <p>Use {@link #builder()} or factory methods to create instances.
     */
    private HighFidelitySwerveSim3DConfig(Builder builder) {
        this.longitudinalCoeffs = builder.longitudinalCoeffs;
        this.lateralCoeffs = builder.lateralCoeffs;
        this.frictionCoefficientLongitudinal = builder.frictionCoefficientLongitudinal;
        this.frictionCoefficientLateral = builder.frictionCoefficientLateral;
        this.wheelRadius = builder.wheelRadius;
        this.wheelWidth = builder.wheelWidth;
        this.contactPatchRadius = builder.contactPatchRadius;
        this.mechanicalTrail = builder.mechanicalTrail;
        this.enableScrubTorque = builder.enableScrubTorque;
        this.scrubFrictionCoefficient = builder.scrubFrictionCoefficient;
        this.scrubVelocityThreshold = builder.scrubVelocityThreshold;
        this.enableSelfAligningTorque = builder.enableSelfAligningTorque;
        this.pneumaticTrailCoefficient = builder.pneumaticTrailCoefficient;
        this.enableLatencySimulation = builder.enableLatencySimulation;
        this.canBusDelay = builder.canBusDelay;
        this.encoderPositionNoiseStdDev = builder.encoderPositionNoiseStdDev;
        this.encoderVelocityNoiseStdDev = builder.encoderVelocityNoiseStdDev;
        this.sensorUpdatePeriod = builder.sensorUpdatePeriod;
    }

    /**
     *
     *
     * <h2>Creates a New Builder.</h2>
     *
     * @return a new Builder instance with default values
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     *
     *
     * <h2>Creates a Builder from This Config.</h2>
     *
     * <p>Useful for creating a modified copy of an existing configuration.
     *
     * @return a new Builder pre-populated with this config's values
     */
    public Builder toBuilder() {
        return new Builder()
                .withPacejkaLongitudinal(longitudinalCoeffs)
                .withPacejkaLateral(lateralCoeffs)
                .withAnisotropicFriction(frictionCoefficientLongitudinal, frictionCoefficientLateral)
                .withWheelGeometry(wheelRadius, wheelWidth, contactPatchRadius)
                .withMechanicalTrail(mechanicalTrail)
                .withScrubTorque(enableScrubTorque, scrubFrictionCoefficient, scrubVelocityThreshold)
                .withSelfAligningTorque(enableSelfAligningTorque, pneumaticTrailCoefficient)
                .withLatency(
                        enableLatencySimulation,
                        canBusDelay,
                        encoderPositionNoiseStdDev,
                        encoderVelocityNoiseStdDev,
                        sensorUpdatePeriod);
    }

    // ==================== Factory Methods ====================

    /**
     *
     *
     * <h2>Creates Default Configuration.</h2>
     *
     * <p>Suitable for typical FRC swerve robots with nitrile tread wheels on carpet. Latency simulation is disabled by
     * default for easier initial testing.
     *
     * <h3>Default Values:</h3>
     *
     * <ul>
     *   <li>Pacejka: Default longitudinal and lateral coefficients
     *   <li>Friction: 1.2 longitudinal, 1.4 lateral (more lateral grip typical for FRC)
     *   <li>Wheel geometry: 2" radius, 1.5" width
     *   <li>Scrub torque: Enabled with 1.0 coefficient
     *   <li>Self-aligning torque: Enabled with 5mm mechanical trail
     *   <li>Latency: Disabled
     * </ul>
     *
     * @return a default configuration
     */
    public static HighFidelitySwerveSim3DConfig defaultConfig() {
        return builder()
                .withPacejkaLongitudinal(PacejkaCoefficients.defaultLongitudinal())
                .withPacejkaLateral(PacejkaCoefficients.defaultLateral())
                .withAnisotropicFriction(1.2, 1.4)
                .withWheelGeometry(Inches.of(2), Inches.of(1.5), Inches.of(0.75))
                .withMechanicalTrail(Millimeters.of(5))
                .withScrubTorque(true, 1.0, MetersPerSecond.of(0.1))
                .withSelfAligningTorque(true, 0.01)
                .withLatencyDisabled()
                .build();
    }

    /**
     *
     *
     * <h2>Creates Competition-Accurate Configuration.</h2>
     *
     * <p>Enables latency and noise simulation for realistic competition conditions. Use this to test control system
     * robustness against real-world sensor imperfections.
     *
     * @return a competition-accurate configuration
     */
    public static HighFidelitySwerveSim3DConfig competitionConfig() {
        return defaultConfig().toBuilder()
                .withLatency(
                        true, Milliseconds.of(15), Radians.of(0.002), RadiansPerSecond.of(0.02), Milliseconds.of(10))
                .build();
    }

    /**
     *
     *
     * <h2>Creates High-Grip Configuration.</h2>
     *
     * <p>For robots with aggressive tread wheels like TPU printed treads or spike treads.
     *
     * @return a high-grip configuration
     */
    public static HighFidelitySwerveSim3DConfig highGripConfig() {
        return builder()
                .withPacejkaLongitudinal(PacejkaCoefficients.aggressiveGrip())
                .withPacejkaLateral(PacejkaCoefficients.aggressiveGrip())
                .withAnisotropicFriction(1.5, 1.8)
                .withWheelGeometry(Inches.of(2), Inches.of(1.5), Inches.of(0.75))
                .withMechanicalTrail(Millimeters.of(5))
                .withScrubTorque(true, 1.2, MetersPerSecond.of(0.1))
                .withSelfAligningTorque(true, 0.012)
                .withLatencyDisabled()
                .build();
    }

    /**
     *
     *
     * <h2>Creates Low-Grip Configuration.</h2>
     *
     * <p>For testing robot behavior on slippery surfaces or with worn wheels.
     *
     * @return a low-grip configuration
     */
    public static HighFidelitySwerveSim3DConfig lowGripConfig() {
        return builder()
                .withPacejkaLongitudinal(PacejkaCoefficients.lowGrip())
                .withPacejkaLateral(PacejkaCoefficients.lowGrip())
                .withAnisotropicFriction(0.8, 0.9)
                .withWheelGeometry(Inches.of(2), Inches.of(1.5), Inches.of(0.75))
                .withMechanicalTrail(Millimeters.of(5))
                .withScrubTorque(true, 0.7, MetersPerSecond.of(0.15))
                .withSelfAligningTorque(true, 0.008)
                .withLatencyDisabled()
                .build();
    }

    // ==================== Builder Class ====================

    /**
     *
     *
     * <h2>Fluent Builder for HighFidelitySwerveSim3DConfig.</h2>
     */
    public static class Builder {
        // Pacejka
        private PacejkaCoefficients longitudinalCoeffs = PacejkaCoefficients.defaultLongitudinal();
        private PacejkaCoefficients lateralCoeffs = PacejkaCoefficients.defaultLateral();

        // Friction
        private double frictionCoefficientLongitudinal = 1.2;
        private double frictionCoefficientLateral = 1.4;

        // Wheel geometry
        private Distance wheelRadius = Inches.of(2);
        private Distance wheelWidth = Inches.of(1.5);
        private Distance contactPatchRadius = Inches.of(0.75);
        private Distance mechanicalTrail = Millimeters.of(5);

        // Scrub torque
        private boolean enableScrubTorque = true;
        private double scrubFrictionCoefficient = 1.0;
        private LinearVelocity scrubVelocityThreshold = MetersPerSecond.of(0.1);

        // Self-aligning torque
        private boolean enableSelfAligningTorque = true;
        private double pneumaticTrailCoefficient = 0.01;

        // Latency
        private boolean enableLatencySimulation = false;
        private Time canBusDelay = Milliseconds.of(15);
        private Angle encoderPositionNoiseStdDev = Radians.of(0.001);
        private AngularVelocity encoderVelocityNoiseStdDev = RadiansPerSecond.of(0.01);
        private Time sensorUpdatePeriod = Milliseconds.of(10);

        /**
         *
         *
         * <h2>Sets Pacejka Longitudinal Coefficients.</h2>
         *
         * @param coeffs the Pacejka coefficients for driving/braking forces
         * @return this builder for chaining
         */
        public Builder withPacejkaLongitudinal(PacejkaCoefficients coeffs) {
            this.longitudinalCoeffs = coeffs;
            return this;
        }

        /**
         *
         *
         * <h2>Sets Pacejka Lateral Coefficients.</h2>
         *
         * @param coeffs the Pacejka coefficients for cornering forces
         * @return this builder for chaining
         */
        public Builder withPacejkaLateral(PacejkaCoefficients coeffs) {
            this.lateralCoeffs = coeffs;
            return this;
        }

        /**
         *
         *
         * <h2>Sets Anisotropic Friction Coefficients.</h2>
         *
         * <p>Different friction in longitudinal (forward/back) vs lateral (sideways) directions. FRC wheels typically
         * have higher lateral grip due to tread pattern.
         *
         * @param longitudinal friction coefficient for driving/braking (typical: 1.0-1.5)
         * @param lateral friction coefficient for cornering (typical: 1.2-1.8)
         * @return this builder for chaining
         */
        public Builder withAnisotropicFriction(double longitudinal, double lateral) {
            this.frictionCoefficientLongitudinal = longitudinal;
            this.frictionCoefficientLateral = lateral;
            return this;
        }

        /**
         *
         *
         * <h2>Sets Isotropic Friction Coefficient.</h2>
         *
         * <p>Same friction in both directions. Use {@link #withAnisotropicFriction(double, double)} for more realistic
         * behavior.
         *
         * @param coefficient the friction coefficient for both directions
         * @return this builder for chaining
         */
        public Builder withFrictionCoefficient(double coefficient) {
            return withAnisotropicFriction(coefficient, coefficient);
        }

        /**
         *
         *
         * <h2>Sets Wheel Geometry.</h2>
         *
         * @param radius wheel radius (affects slip calculation)
         * @param width wheel width (affects scrub torque)
         * @param contactPatch contact patch radius for scrub calculation
         * @return this builder for chaining
         */
        public Builder withWheelGeometry(Distance radius, Distance width, Distance contactPatch) {
            this.wheelRadius = radius;
            this.wheelWidth = width;
            this.contactPatchRadius = contactPatch;
            return this;
        }

        /**
         *
         *
         * <h2>Sets Mechanical Trail.</h2>
         *
         * <p>The distance behind the steering axis where the contact patch is located. Creates self-aligning torque
         * from lateral forces.
         *
         * @param trail mechanical trail distance (typical: 3-10mm for FRC modules)
         * @return this builder for chaining
         */
        public Builder withMechanicalTrail(Distance trail) {
            this.mechanicalTrail = trail;
            return this;
        }

        /**
         *
         *
         * <h2>Configures Scrub Torque.</h2>
         *
         * <p>Scrub torque is the resistance to steering when the wheel is not rolling. It's caused by the wheel
         * "scrubbing" against the carpet as it pivots.
         *
         * @param enabled whether to simulate scrub torque
         * @param frictionCoefficient scrub friction coefficient (typically same as lateral friction)
         * @param velocityThreshold velocity below which scrub torque applies
         * @return this builder for chaining
         */
        public Builder withScrubTorque(boolean enabled, double frictionCoefficient, LinearVelocity velocityThreshold) {
            this.enableScrubTorque = enabled;
            this.scrubFrictionCoefficient = frictionCoefficient;
            this.scrubVelocityThreshold = velocityThreshold;
            return this;
        }

        /**
         *
         *
         * <h2>Disables Scrub Torque.</h2>
         *
         * @return this builder for chaining
         */
        public Builder withScrubTorqueDisabled() {
            this.enableScrubTorque = false;
            return this;
        }

        /**
         *
         *
         * <h2>Configures Self-Aligning Torque.</h2>
         *
         * <p>Self-aligning torque is created when lateral forces act through the pneumatic and mechanical trail,
         * creating a torque that tries to straighten the wheel.
         *
         * @param enabled whether to simulate self-aligning torque
         * @param pneumaticTrailCoeff load-dependent pneumatic trail coefficient
         * @return this builder for chaining
         */
        public Builder withSelfAligningTorque(boolean enabled, double pneumaticTrailCoeff) {
            this.enableSelfAligningTorque = enabled;
            this.pneumaticTrailCoefficient = pneumaticTrailCoeff;
            return this;
        }

        /**
         *
         *
         * <h2>Disables Self-Aligning Torque.</h2>
         *
         * @return this builder for chaining
         */
        public Builder withSelfAligningTorqueDisabled() {
            this.enableSelfAligningTorque = false;
            return this;
        }

        /**
         *
         *
         * <h2>Configures Latency Simulation.</h2>
         *
         * <p>Simulates real-world imperfections: CAN bus command delay and encoder noise.
         *
         * @param enabled whether to simulate latency and noise
         * @param canDelay CAN bus round-trip delay (typical: 10-20ms)
         * @param positionNoise encoder position noise standard deviation
         * @param velocityNoise encoder velocity noise standard deviation
         * @param sensorPeriod sensor update period
         * @return this builder for chaining
         */
        public Builder withLatency(
                boolean enabled, Time canDelay, Angle positionNoise, AngularVelocity velocityNoise, Time sensorPeriod) {
            this.enableLatencySimulation = enabled;
            this.canBusDelay = canDelay;
            this.encoderPositionNoiseStdDev = positionNoise;
            this.encoderVelocityNoiseStdDev = velocityNoise;
            this.sensorUpdatePeriod = sensorPeriod;
            return this;
        }

        /**
         *
         *
         * <h2>Disables Latency Simulation.</h2>
         *
         * @return this builder for chaining
         */
        public Builder withLatencyDisabled() {
            this.enableLatencySimulation = false;
            return this;
        }

        /**
         *
         *
         * <h2>Builds the Configuration.</h2>
         *
         * @return the configured HighFidelitySwerveSim3DConfig instance
         */
        public HighFidelitySwerveSim3DConfig build() {
            return new HighFidelitySwerveSim3DConfig(this);
        }
    }
}
