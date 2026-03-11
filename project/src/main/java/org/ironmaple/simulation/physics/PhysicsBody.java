package org.ironmaple.simulation.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 *
 *
 * <h1>Physics Body Abstraction</h1>
 *
 * <p>Represents a rigid body in the 3D physics simulation. This interface abstracts the underlying physics engine
 * (Bullet, dyn4j, etc.) so the simulation logic is decoupled from the specific implementation.
 *
 * <p>Each physics body has:
 *
 * <ul>
 *   <li>A 3D pose (position + rotation)
 *   <li>Linear and angular velocity
 *   <li>Mass properties (mass, inertia tensor)
 *   <li>The ability to receive forces and torques
 * </ul>
 */
public interface PhysicsBody {

    /**
     *
     *
     * <h2>Gets the Current 3D Pose of This Body.</h2>
     *
     * @return the pose as a {@link Pose3d}
     */
    Pose3d getPose3d();

    /**
     *
     *
     * <h2>Sets the 3D Pose of This Body.</h2>
     *
     * <p>This teleports the body to the specified pose instantly.
     *
     * @param pose the new pose
     */
    void setPose3d(Pose3d pose);

    /**
     *
     *
     * <h2>Gets the Linear Velocity of This Body.</h2>
     *
     * @return the linear velocity in meters per second, as a {@link Translation3d}
     */
    Translation3d getLinearVelocityMPS();

    /**
     *
     *
     * <h2>Sets the Linear Velocity of This Body.</h2>
     *
     * @param velocityMPS the linear velocity in meters per second
     */
    void setLinearVelocityMPS(Translation3d velocityMPS);

    /**
     *
     *
     * <h2>Gets the Angular Velocity of This Body.</h2>
     *
     * @return the angular velocity in radians per second, as a vector {@link Translation3d} (x=roll rate, y=pitch rate,
     *     z=yaw rate)
     */
    Translation3d getAngularVelocityRadPerSec();

    /**
     *
     *
     * <h2>Sets the Angular Velocity of This Body.</h2>
     *
     * @param angularVelocityRadPerSec the angular velocity vector
     */
    void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec);

    /**
     *
     *
     * <h2>Applies a Force to the Center of Mass.</h2>
     *
     * <p>The force is applied in world coordinates.
     *
     * @param forceNewtons the force vector in Newtons
     */
    void applyForce(Translation3d forceNewtons);

    /**
     *
     *
     * <h2>Applies a Force at a Specific Point.</h2>
     *
     * <p>This can create both linear acceleration and torque depending on the offset from the center of mass.
     *
     * @param forceNewtons the force vector in Newtons (world coordinates)
     * @param pointWorld the point of application in world coordinates
     */
    void applyForceAtPoint(Translation3d forceNewtons, Translation3d pointWorld);

    /**
     *
     *
     * <h2>Applies a Torque to This Body.</h2>
     *
     * @param torqueNewtonMeters the torque vector in Newton-meters
     */
    void applyTorque(Translation3d torqueNewtonMeters);

    /**
     *
     *
     * <h2>Gets the Mass of This Body.</h2>
     *
     * @return the mass in kilograms
     */
    double getMassKg();

    /**
     *
     *
     * <h2>Checks if This Body is Static (Immovable).</h2>
     *
     * @return true if the body is static (e.g., a wall), false if dynamic
     */
    boolean isStatic();

    /**
     *
     *
     * <h2>Gets the Linear Velocity at a Specific World Point.</h2>
     *
     * <p>This accounts for both the body's linear velocity and its angular velocity contribution at the given point.
     *
     * @param pointWorld the point in world coordinates
     * @return the velocity at that point in meters per second
     */
    Translation3d getLinearVelocityAtPointMPS(Translation3d pointWorld);

    /**
     *
     *
     * <h2>Sets user data associated with this body.</h2>
     *
     * @param data the user data object
     */
    void setUserData(Object data);

    /**
     *
     *
     * <h2>Gets user data associated with this body.</h2>
     *
     * @return the user data object
     */
    Object getUserData();

    /**
     *
     *
     * <h2>Sets Linear and Angular Damping.</h2>
     *
     * <p>Damping applies a velocity-proportional resistance force. Higher values slow the body faster.
     *
     * @param linearDamping linear velocity damping coefficient (0 = no damping, 1 = high damping)
     * @param angularDamping angular velocity damping coefficient
     */
    void setDamping(double linearDamping, double angularDamping);

    /**
     *
     *
     * <h2>Sets the Friction Coefficient.</h2>
     *
     * <p>Higher values = more grip.
     *
     * @param friction the friction coefficient (typically 0.0 - 1.0+)
     */
    void setFriction(double friction);

    /**
     *
     *
     * <h2>Sets the Restitution (Bounciness).</h2>
     *
     * <p>0.0 = no bounce, 1.0 = perfect bounce.
     *
     * @param restitution the restitution coefficient
     */
    void setRestitution(double restitution);

    int getBodyId();

    /**
     *
     *
     * <h2>Sets whether this body should report contact events.</h2>
     *
     * <p>Used for performance optimization. High-fidelity bodies (Tier 1) should enable this. Bulk/Background bodies
     * (Tier 2) should disable it.
     *
     * @param enable true to enable contact reporting
     */
    void setContactReporting(boolean enable);

    /**
     *
     *
     * <h2>Sets the collision layer for this body.</h2>
     *
     * <p>Used to control which objects this body collides with.
     *
     * @param layer the collision layer index
     */
    void setCollisionLayer(int layer);
}
