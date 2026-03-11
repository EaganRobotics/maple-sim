package org.ironmaple.simulation.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

/**
 *
 *
 * <h1>Physics Engine Abstraction</h1>
 *
 * <p>Represents a 3D physics simulation world. This interface abstracts the underlying physics engine (Bullet, etc.) so
 * that the simulation logic is decoupled from the specific implementation.
 *
 * <p>The physics engine manages:
 *
 * <ul>
 *   <li>A collection of {@link PhysicsBody} objects
 *   <li>Collision detection and response
 *   <li>Time stepping
 *   <li>Raycasting for suspension/ground detection
 * </ul>
 */
public interface PhysicsEngine {

    /**
     *
     *
     * <h2>Initializes the Physics Engine.</h2>
     *
     * <p>Must be called before any other methods. Sets up the native physics world.
     */
    void initialize();

    /**
     *
     *
     * <h2>Shuts Down the Physics Engine.</h2>
     *
     * <p>Releases all native resources. The engine cannot be used after this call.
     */
    void shutdown();

    /**
     *
     *
     * <h2>Steps the Physics Simulation Forward.</h2>
     *
     * @param deltaTime the time step duration
     */
    void step(Time deltaTime);

    /**
     *
     *
     * <h2>Creates a Dynamic (Movable) Body.</h2>
     *
     * @param shape the collision shape
     * @param massKg the mass in kilograms
     * @param initialPose the initial pose
     * @return the created physics body
     */
    PhysicsBody createDynamicBody(PhysicsShape shape, double massKg, Pose3d initialPose);

    /**
     *
     *
     * <h2>Creates a Dynamic (Movable) Body with Specific Physical Properties.</h2>
     *
     * @param shape the collision shape
     * @param massKg the mass in kilograms
     * @param friction the friction coefficient
     * @param restitution the coefficient of restitution (bounciness)
     * @param linearDamping the linear damping coefficient
     * @param angularDamping the angular damping coefficient
     * @param initialPose the initial pose
     * @return the created physics body
     */
    PhysicsBody createDynamicBody(
            PhysicsShape shape,
            double massKg,
            double friction,
            double restitution,
            double linearDamping,
            double angularDamping,
            Pose3d initialPose);

    /**
     *
     *
     * <h2>Creates a Static (Immovable) Body.</h2>
     *
     * <p>Static bodies are used for walls, field elements, etc.
     *
     * @param shape the collision shape
     * @param pose the pose of the static body
     * @return the created physics body
     */
    PhysicsBody createStaticBody(PhysicsShape shape, Pose3d pose);

    /**
     *
     *
     * <h2>Creates a Kinematic Body.</h2>
     *
     * <p>Kinematic bodies are moved by code (setting velocity/position) but push dynamic bodies. They essentially have
     * infinite mass but move according to script.
     *
     * @param shape the collision shape
     * @param pose the initial pose
     * @return the created physics body
     */
    PhysicsBody createKinematicBody(PhysicsShape shape, Pose3d pose);

    /**
     *
     *
     * <h2>Creates a compound shape with an offset.</h2>
     *
     * <p>Wraps the given shape in a compound shape that applies the specified translation offset. This is useful for
     * offsetting the Center of Mass relative to the visual/collision center.
     *
     * @param shape the base shape
     * @param offset the translation offset to apply to the shape
     * @return a new physics shape representing the offset shape
     */
    PhysicsShape createOffsetShape(PhysicsShape shape, Translation3d offset);

    /**
     *
     *
     * <h2>Removes a Body from the Physics World.</h2>
     *
     * @param body the body to remove
     */
    void removeBody(PhysicsBody body);

    /**
     *
     *
     * <h2>Removes All Bodies from the Physics World.</h2>
     */
    void removeAllBodies();

    /**
     * Adds an existing body to the physics world.
     *
     * @param body the body to add
     */
    void addBody(PhysicsBody body);

    /**
     * Gets all bodies in the physics world.
     *
     * @return list of all bodies
     */
    java.util.List<PhysicsBody> getBodies();

    /**
     *
     *
     * <h2>Creates a Box Shape.</h2>
     *
     * @param halfExtentsMeters the half-extents (half width, half height, half depth)
     * @return the created shape
     */
    PhysicsShape createBoxShape(Translation3d halfExtentsMeters);

    /**
     *
     *
     * <h2>Creates a Sphere Shape.</h2>
     *
     * @param radiusMeters the radius in meters
     * @return the created shape
     */
    PhysicsShape createSphereShape(double radiusMeters);

    /**
     *
     *
     * <h2>Creates a Cylinder Shape.</h2>
     *
     * @param radiusMeters the radius in meters
     * @param heightMeters the height in meters
     * @return the created shape
     */
    PhysicsShape createCylinderShape(double radiusMeters, double heightMeters);

    /**
     *
     *
     * <h2>Creates a Convex Hull Shape from Vertices.</h2>
     *
     * @param vertices the vertices defining the hull
     * @return the created shape
     */
    PhysicsShape createConvexHullShape(Translation3d[] vertices);

    /**
     *
     *
     * <h2>Creates a Compound Shape from an OBJ Mesh.</h2>
     *
     * <p>Parses the OBJ file at the given resource path. Each object/group in the OBJ is treated as a separate convex
     * hull.
     *
     * @param resourcePath the classpath resource path to the OBJ file
     * @return the created compound shape
     */
    PhysicsShape createCompoundShapeFromMesh(String resourcePath);

    /**
     *
     *
     * <h2>Performs a Raycast.</h2>
     *
     * <p>Casts a ray from the origin in the specified direction and returns the closest hit.
     *
     * @param origin the ray origin in world coordinates
     * @param direction the ray direction (will be normalized)
     * @param maxDistance the maximum distance to check
     * @return the raycast result, or empty if no hit
     */
    Optional<RaycastResult> raycast(Translation3d origin, Translation3d direction, double maxDistance);

    /**
     * Performs a raycast excluding a specific body (useful for avoiding self-collision).
     *
     * @param origin the ray origin in world coordinates
     * @param direction the ray direction (will be normalized)
     * @param maxDistance the maximum distance to check
     * @param excludeBody the body to exclude from raycast results (can be null)
     * @return the raycast result, or empty if no hit
     */
    default Optional<RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance, PhysicsBody excludeBody) {
        // Default implementation ignores excludeBody - subclasses should override
        return raycast(origin, direction, maxDistance);
    }

    /**
     *
     *
     * <h2>Sets the Global Gravity.</h2>
     *
     * @param gravityMPS2 the gravity vector in meters per second squared
     */
    void setGravity(Translation3d gravityMPS2);

    /**
     *
     *
     * <h2>Finds all bodies overlapping with the given shape at the given pose.</h2>
     *
     * @param shape the shape to check for overlaps
     * @param pose the pose of the shape
     * @return a list of overlapping physics bodies
     */
    java.util.List<PhysicsBody> getOverlappingBodies(PhysicsShape shape, Pose3d pose);

    /**
     * Result of a raycast operation.
     *
     * @param hitPoint the point of impact in world coordinates
     * @param hitNormal the surface normal at the hit point
     * @param hitDistance the distance from the ray origin to the hit point
     * @param hitBody the body that was hit
     */
    record RaycastResult(Translation3d hitPoint, Translation3d hitNormal, double hitDistance, PhysicsBody hitBody) {}
}
