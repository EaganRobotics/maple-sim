package org.ironmaple.simulation.physics.dyn4j;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.ironmaple.simulation.physics.PhysicsBackend;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Dyn4j Physics Backend (2D)</h1>
 *
 * <p>Implements the {@link PhysicsBackend} interface using the dyn4j 2D physics engine. This is the original physics
 * backend used by maple-sim.
 */
public class Dyn4jBackend implements PhysicsBackend {
    private World<Body> world;
    private boolean initialized = false;

    @Override
    public boolean is3D() {
        return false;
    }

    @Override
    public void initialize() {
        if (initialized) return;
        world = new World<>();
        world.setGravity(PhysicsWorld.ZERO_GRAVITY);
        initialized = true;
    }

    @Override
    public void shutdown() {
        if (!initialized) return;
        world.removeAllBodies();
        world = null;
        initialized = false;
    }

    @Override
    public void step(Time deltaTime) {
        ensureInitialized();
        world.step(1, deltaTime.in(Seconds));
    }

    @Override
    public Object addStaticBox(Translation3d halfExtents, Pose3d pose) {
        ensureInitialized();
        // Project to 2D (ignore Z)
        double width = halfExtents.getX() * 2;
        double height = halfExtents.getY() * 2;

        Body obstacle = new Body();
        obstacle.setMass(MassType.INFINITE);
        BodyFixture fixture = obstacle.addFixture(Geometry.createRectangle(width, height));
        fixture.setFriction(0.6);
        fixture.setRestitution(0.3);

        // Set 2D pose (extract yaw from 3D rotation)
        Pose2d pose2d = new Pose2d(
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getRotation().toRotation2d());
        obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose2d));

        world.addBody(obstacle);
        return obstacle;
    }

    @Override
    public Object addStaticLine(Translation2d start, Translation2d end) {
        ensureInitialized();

        Body obstacle = new Body();
        obstacle.setMass(MassType.INFINITE);
        BodyFixture fixture = obstacle.addFixture(
                Geometry.createSegment(GeometryConvertor.toDyn4jVector2(start), GeometryConvertor.toDyn4jVector2(end)));
        fixture.setFriction(0.6);
        fixture.setRestitution(0.3);

        world.addBody(obstacle);
        return obstacle;
    }

    @Override
    public void removeBody(Object bodyHandle) {
        ensureInitialized();
        if (bodyHandle instanceof Body body) {
            world.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        ensureInitialized();
        world.removeAllBodies();
    }

    @Override
    public Optional<PhysicsEngine.RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance) {
        // 2D backend doesn't support raycasting in the same way
        return Optional.empty();
    }

    @Override
    public void setGravity(Translation3d gravity) {
        ensureInitialized();
        // 2D world uses zero gravity (horizontal plane simulation)
        world.setGravity(PhysicsWorld.ZERO_GRAVITY);
    }

    /**
     *
     *
     * <h2>Gets the Underlying Dyn4j World.</h2>
     *
     * <p>For advanced use cases that need direct access to dyn4j features.
     *
     * @return the dyn4j world
     */
    public World<Body> getWorld() {
        ensureInitialized();
        return world;
    }

    /**
     *
     *
     * <h2>Adds a Body Directly to the World.</h2>
     *
     * <p>This is used by existing simulation classes that extend dyn4j Body.
     *
     * @param body the body to add
     */
    public void addBody(Body body) {
        ensureInitialized();
        world.addBody(body);
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("Dyn4jBackend has not been initialized. Call initialize() first.");
        }
    }
}
