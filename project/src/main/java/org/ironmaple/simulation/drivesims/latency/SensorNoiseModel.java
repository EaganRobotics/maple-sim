package org.ironmaple.simulation.drivesims.latency;

import java.util.Random;

/**
 *
 *
 * <h1>Gaussian Noise Model for Sensor Readings.</h1>
 *
 * <p>Adds configurable Gaussian noise to encoder position and velocity measurements. Used to simulate real-world sensor
 * imperfections in high-fidelity swerve simulations.
 *
 * <h2>Typical Noise Values for FRC Encoders:</h2>
 *
 * <ul>
 *   <li>Position noise: 0.001 - 0.005 radians (depends on encoder resolution)
 *   <li>Velocity noise: 0.01 - 0.05 rad/s (velocity is derived, more noise)
 * </ul>
 *
 * <h2>Usage:</h2>
 *
 * <pre>
 * SensorNoiseModel noise = new SensorNoiseModel(0.002, 0.02);
 *
 * // Add noise to readings
 * double noisyPosition = noise.addPositionNoise(truePosition);
 * double noisyVelocity = noise.addVelocityNoise(trueVelocity);
 * </pre>
 */
public class SensorNoiseModel {

    private final Random random;
    private final double positionStdDevRadians;
    private final double velocityStdDevRadPerSec;

    /**
     *
     *
     * <h2>Constructs a Sensor Noise Model.</h2>
     *
     * @param positionStdDevRadians standard deviation of position noise in radians
     * @param velocityStdDevRadPerSec standard deviation of velocity noise in rad/s
     */
    public SensorNoiseModel(double positionStdDevRadians, double velocityStdDevRadPerSec) {
        this.random = new Random();
        this.positionStdDevRadians = positionStdDevRadians;
        this.velocityStdDevRadPerSec = velocityStdDevRadPerSec;
    }

    /**
     *
     *
     * <h2>Constructs a Sensor Noise Model with Custom Random Seed.</h2>
     *
     * <p>Use this constructor for reproducible noise patterns in testing.
     *
     * @param positionStdDevRadians standard deviation of position noise in radians
     * @param velocityStdDevRadPerSec standard deviation of velocity noise in rad/s
     * @param seed random seed for reproducibility
     */
    public SensorNoiseModel(double positionStdDevRadians, double velocityStdDevRadPerSec, long seed) {
        this.random = new Random(seed);
        this.positionStdDevRadians = positionStdDevRadians;
        this.velocityStdDevRadPerSec = velocityStdDevRadPerSec;
    }

    /**
     *
     *
     * <h2>Adds Noise to Position Reading.</h2>
     *
     * @param positionRadians the true position in radians
     * @return the noisy position (true + Gaussian noise)
     */
    public double addPositionNoise(double positionRadians) {
        if (positionStdDevRadians <= 0) return positionRadians;
        return positionRadians + random.nextGaussian() * positionStdDevRadians;
    }

    /**
     *
     *
     * <h2>Adds Noise to Velocity Reading.</h2>
     *
     * @param velocityRadPerSec the true velocity in rad/s
     * @return the noisy velocity (true + Gaussian noise)
     */
    public double addVelocityNoise(double velocityRadPerSec) {
        if (velocityStdDevRadPerSec <= 0) return velocityRadPerSec;
        return velocityRadPerSec + random.nextGaussian() * velocityStdDevRadPerSec;
    }

    /**
     *
     *
     * <h2>Sets Random Seed for Reproducibility.</h2>
     *
     * <p>Call this before a test sequence to get reproducible noise patterns.
     *
     * @param seed the random seed
     */
    public void setSeed(long seed) {
        random.setSeed(seed);
    }

    /**
     *
     *
     * <h2>Gets the Position Noise Standard Deviation.</h2>
     *
     * @return position noise in radians
     */
    public double getPositionStdDevRadians() {
        return positionStdDevRadians;
    }

    /**
     *
     *
     * <h2>Gets the Velocity Noise Standard Deviation.</h2>
     *
     * @return velocity noise in rad/s
     */
    public double getVelocityStdDevRadPerSec() {
        return velocityStdDevRadPerSec;
    }

    /**
     *
     *
     * <h2>Creates a Zero-Noise Model.</h2>
     *
     * <p>Returns a model that adds no noise. Useful as a default when noise is disabled.
     *
     * @return a SensorNoiseModel with zero standard deviations
     */
    public static SensorNoiseModel noNoise() {
        return new SensorNoiseModel(0, 0);
    }

    /**
     *
     *
     * <h2>Creates a Typical FRC Encoder Noise Model.</h2>
     *
     * <p>Based on typical noise levels observed in FRC motor encoders.
     *
     * @return a SensorNoiseModel with typical FRC values
     */
    public static SensorNoiseModel typicalFrcEncoder() {
        return new SensorNoiseModel(0.002, 0.02);
    }

    /**
     *
     *
     * <h2>Creates a High-Resolution Encoder Noise Model.</h2>
     *
     * <p>For high-resolution encoders like CANCoders or through-bore encoders.
     *
     * @return a SensorNoiseModel with lower noise values
     */
    public static SensorNoiseModel highResolutionEncoder() {
        return new SensorNoiseModel(0.0005, 0.005);
    }
}
