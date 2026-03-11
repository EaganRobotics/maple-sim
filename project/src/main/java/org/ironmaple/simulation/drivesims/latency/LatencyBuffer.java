package org.ironmaple.simulation.drivesims.latency;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 *
 *
 * <h1>Generic Ring Buffer for Latency Simulation.</h1>
 *
 * <p>Stores timestamped values and retrieves them after a configurable delay. Used to simulate CAN bus communication
 * delay in high-fidelity swerve simulations.
 *
 * <h2>Usage:</h2>
 *
 * <pre>
 * // Create buffer with 15ms delay
 * LatencyBuffer&lt;SwerveModuleState&gt; buffer = new LatencyBuffer&lt;&gt;(0.015, initialState);
 *
 * // Each simulation tick, add current command
 * buffer.add(currentTime, commandedState);
 *
 * // Get the delayed state (what the motor actually sees)
 * SwerveModuleState delayedState = buffer.get(currentTime);
 * </pre>
 *
 * @param <T> the type of value to buffer
 */
public class LatencyBuffer<T> {

    private record TimestampedValue<T>(double timestampSeconds, T value) {}

    private final Deque<TimestampedValue<T>> buffer = new ArrayDeque<>();
    private final double delaySeconds;
    private T lastOutput;

    /**
     *
     *
     * <h2>Constructs a Latency Buffer.</h2>
     *
     * @param delaySeconds the delay in seconds (typical: 0.010 to 0.020 for CAN bus)
     * @param initialValue the initial value returned before any data arrives
     */
    public LatencyBuffer(double delaySeconds, T initialValue) {
        this.delaySeconds = delaySeconds;
        this.lastOutput = initialValue;
    }

    /**
     *
     *
     * <h2>Adds a Value to the Buffer.</h2>
     *
     * <p>Call this every simulation tick with the current command/measurement.
     *
     * @param currentTimeSeconds current simulation time in seconds
     * @param value the value to add
     */
    public void add(double currentTimeSeconds, T value) {
        buffer.addLast(new TimestampedValue<>(currentTimeSeconds, value));

        // Remove old entries beyond max buffer time (delay + some margin)
        double cutoffTime = currentTimeSeconds - delaySeconds - 0.1;
        while (!buffer.isEmpty() && buffer.peekFirst().timestampSeconds < cutoffTime) {
            buffer.pollFirst();
        }
    }

    /**
     *
     *
     * <h2>Gets the Delayed Value.</h2>
     *
     * <p>Returns the value from (currentTime - delay). If no value exists at that time, returns the most recent value
     * before that time, or the initial value if the buffer is empty.
     *
     * @param currentTimeSeconds current simulation time in seconds
     * @return the delayed value
     */
    public T get(double currentTimeSeconds) {
        double targetTime = currentTimeSeconds - delaySeconds;

        // Find the most recent value at or before targetTime
        T result = lastOutput;
        for (TimestampedValue<T> entry : buffer) {
            if (entry.timestampSeconds <= targetTime) {
                result = entry.value;
            } else {
                break;
            }
        }

        lastOutput = result;
        return result;
    }

    /**
     *
     *
     * <h2>Gets the Delay Duration.</h2>
     *
     * @return the configured delay in seconds
     */
    public double getDelaySeconds() {
        return delaySeconds;
    }

    /**
     *
     *
     * <h2>Gets the Buffer Size.</h2>
     *
     * @return the number of entries currently in the buffer
     */
    public int size() {
        return buffer.size();
    }

    /**
     *
     *
     * <h2>Clears the Buffer.</h2>
     *
     * <p>Removes all stored values. The next {@link #get(double)} call will return the last known value until new data
     * is added.
     */
    public void clear() {
        buffer.clear();
    }

    /**
     *
     *
     * <h2>Resets the Buffer with a New Initial Value.</h2>
     *
     * @param initialValue the new initial value
     */
    public void reset(T initialValue) {
        buffer.clear();
        lastOutput = initialValue;
    }
}
