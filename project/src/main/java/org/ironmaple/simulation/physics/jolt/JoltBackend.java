package org.ironmaple.simulation.physics.jolt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import org.ironmaple.simulation.physics.PhysicsBackend;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Jolt Physics Backend Implementation</h1>
 *
 * <p>Implements the {@link PhysicsBackend} interface using Jolt Physics.
 *
 * <p>This provides a simplified interface for obstacle management while leveraging Jolt's 3D physics.
 */
public class JoltBackend implements PhysicsBackend {
    private final JoltPhysicsEngine engine;
    private boolean initialized = false;

    /** Creates a Jolt backend with default configuration. */
    public JoltBackend() {
        this.engine = new JoltPhysicsEngine();
    }

    @Override
    public boolean is3D() {
        return true;
    }

    @Override
    public void initialize() {
        if (initialized) return;

        // Ensure native library is loaded so we can create shapes on main thread
        JoltPhysicsEngine.loadLibrary();

        // Initialize engine on main thread
        engine.initialize();

        initialized = true;
    }

    @Override
    public void shutdown() {
        if (!initialized) return;

        engine.shutdown();
        initialized = false;
    }

    @Override
    public void step(Time deltaTime) {
        ensureInitialized();
        // Synchronous mode: step the engine directly
        engine.step(deltaTime);
    }

    @Override
    public Object addStaticBox(Translation3d halfExtents, Pose3d pose) {
        ensureInitialized();
        PhysicsShape shape = engine.createBoxShape(halfExtents);
        return engine.createStaticBody(shape, pose);
    }

    @Override
    public Object addStaticLine(Translation2d start, Translation2d end) {
        ensureInitialized();
        // Create a thin box to approximate a 2D line in 3D
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double length = Math.sqrt(dx * dx + dy * dy);
        double angle = Math.atan2(dy, dx);

        // Create box shape (thin wall)
        double thickness = 0.02; // 2 cm thick
        double height = 0.5; // 50 cm tall
        Translation3d halfExtents = new Translation3d(length / 2, thickness / 2, height / 2);
        PhysicsShape shape = engine.createBoxShape(halfExtents);

        // Position at midpoint
        double midX = (start.getX() + end.getX()) / 2;
        double midY = (start.getY() + end.getY()) / 2;
        // Jolt rotation is Z-up so this is standard Z rotation
        Pose3d pose = new Pose3d(new Translation3d(midX, midY, height / 2), new Rotation3d(0, 0, angle));

        return engine.createStaticBody(shape, pose);
    }

    @Override
    public void removeBody(Object bodyHandle) {
        ensureInitialized();
        if (bodyHandle instanceof PhysicsBody body) {
            engine.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        ensureInitialized();
        engine.removeAllBodies();
    }

    @Override
    public Optional<PhysicsEngine.RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance) {
        ensureInitialized();
        return engine.raycast(origin, direction, maxDistance);
    }

    @Override
    public void setGravity(Translation3d gravity) {
        ensureInitialized();
        engine.setGravity(gravity);
    }

    /**
     * Gets the underlying Jolt physics engine.
     *
     * @return the Jolt physics engine
     */
    public PhysicsEngine getEngine() {
        ensureInitialized();
        return engine;
    }

    /**
     * Sets the number of worker threads for the physics engine.
     *
     * <p>Must be called before initialize().
     *
     * @param numWorkerThreads number of threads
     */
    public void setWorkerThreadCount(int numWorkerThreads) {
        if (initialized) {
            throw new IllegalStateException("Cannot set worker thread count after initialization");
        }
        engine.setWorkerThreadCount(numWorkerThreads);
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("JoltBackend has not been initialized. Call initialize() first.");
        }
    }
}
