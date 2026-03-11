package org.ironmaple.simulation.physics.jolt;

/**
 *
 *
 * <h1>Jolt Physics Configuration</h1>
 *
 * <p>Configuration settings for the Jolt physics engine.
 */
public class JoltConfig {
    /** Number of worker threads for Jolt's internal JobSystemThreadPool. */
    private int numWorkerThreads;

    /** Maximum number of bodies in the physics system. */
    private int maxBodies = 5_000;

    /** Maximum number of body pairs for collision detection. */
    private int maxBodyPairs = 65_536;

    /** Maximum number of contact points. */
    private int maxContacts = 20_480;

    /** Creates a default configuration using all available processors. */
    public JoltConfig() {
        this.numWorkerThreads = Runtime.getRuntime().availableProcessors();
    }

    /**
     * Creates a configuration with a specific number of worker threads.
     *
     * @param numWorkerThreads number of threads for physics calculations
     */
    public JoltConfig(int numWorkerThreads) {
        this.numWorkerThreads = numWorkerThreads;
    }

    public int getNumWorkerThreads() {
        return numWorkerThreads;
    }

    public JoltConfig setNumWorkerThreads(int numWorkerThreads) {
        this.numWorkerThreads = numWorkerThreads;
        return this;
    }

    public int getMaxBodies() {
        return maxBodies;
    }

    public JoltConfig setMaxBodies(int maxBodies) {
        this.maxBodies = maxBodies;
        return this;
    }

    public int getMaxBodyPairs() {
        return maxBodyPairs;
    }

    public JoltConfig setMaxBodyPairs(int maxBodyPairs) {
        this.maxBodyPairs = maxBodyPairs;
        return this;
    }

    public int getMaxContacts() {
        return maxContacts;
    }

    public JoltConfig setMaxContacts(int maxContacts) {
        this.maxContacts = maxContacts;
        return this;
    }

    /**
     * Creates a default configuration suitable for typical FRC simulation.
     *
     * @return default configuration
     */
    public static JoltConfig defaultConfig() {
        return new JoltConfig();
    }

    /**
     * Creates a lightweight configuration for resource-constrained environments.
     *
     * @return lightweight configuration with fewer threads and lower limits
     */
    public static JoltConfig lightweightConfig() {
        return new JoltConfig(2).setMaxBodies(1_000).setMaxBodyPairs(10_000).setMaxContacts(5_000);
    }
}
