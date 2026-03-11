package org.ironmaple.simulation.physics.bullet;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.concurrent.atomic.AtomicInteger;
import org.ironmaple.simulation.physics.PhysicsBody;

/**
 *
 *
 * <h1>Bullet Physics Body Implementation</h1>
 *
 * <p>Implements the {@link PhysicsBody} interface by wrapping a Bullet {@link PhysicsRigidBody}.
 */
public class BulletBody implements PhysicsBody {
    /** Atomic counter for generating unique body IDs. */
    private static final AtomicInteger ID_COUNTER = new AtomicInteger(0);

    private final PhysicsRigidBody rigidBody;
    private final boolean isStatic;
    private final int bodyId;
    private Object userData;

    /**
     *
     *
     * <h2>Creates a BulletBody Wrapper.</h2>
     *
     * @param rigidBody the underlying Bullet rigid body
     * @param isStatic whether this body is static (mass = 0)
     */
    public BulletBody(PhysicsRigidBody rigidBody, boolean isStatic) {
        this.rigidBody = rigidBody;
        this.isStatic = isStatic;
        this.bodyId = ID_COUNTER.incrementAndGet();
    }

    /**
     *
     *
     * <h2>Gets the Unique Body ID.</h2>
     *
     * <p>Used for tracking bodies across thread boundaries in threaded physics mode.
     *
     * @return the unique body ID
     */
    public int getBodyId() {
        return bodyId;
    }

    /**
     *
     *
     * <h2>Gets the Underlying Bullet RigidBody.</h2>
     *
     * @return the Bullet rigid body
     */
    public PhysicsRigidBody getRigidBody() {
        return rigidBody;
    }

    @Override
    public Pose3d getPose3d() {
        Vector3f location = rigidBody.getPhysicsLocation(null);
        Quaternion rotation = rigidBody.getPhysicsRotation(null);
        return BulletPhysicsEngine.toPose3d(location, rotation);
    }

    @Override
    public void setPose3d(Pose3d pose) {
        rigidBody.setPhysicsLocation(BulletPhysicsEngine.toVector3f(pose.getTranslation()));
        rigidBody.setPhysicsRotation(BulletPhysicsEngine.toQuaternion(pose.getRotation()));
    }

    @Override
    public Translation3d getLinearVelocityMPS() {
        Vector3f velocity = rigidBody.getLinearVelocity(null);
        return BulletPhysicsEngine.toTranslation3d(velocity);
    }

    @Override
    public void setLinearVelocityMPS(Translation3d velocityMPS) {
        rigidBody.setLinearVelocity(BulletPhysicsEngine.toVector3f(velocityMPS));
    }

    @Override
    public Translation3d getAngularVelocityRadPerSec() {
        Vector3f angVel = rigidBody.getAngularVelocity(null);
        return BulletPhysicsEngine.toTranslation3d(angVel).times(-1);
    }

    /**
     * Gets the raw angular velocity Z component (yaw rate) directly.
     *
     * @return the yaw rate in radians per second
     */
    public double getRawAngularVelocityZ() {
        Vector3f angVel = rigidBody.getAngularVelocity(null);
        return -angVel.z;
    }

    @Override
    public void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec) {
        // Extract angular velocity components from Translation3d
        Vector3f angVel = BulletPhysicsEngine.toVector3f(angularVelocityRadPerSec.times(-1));
        rigidBody.setAngularVelocity(angVel);
    }

    @Override
    public void applyForce(Translation3d forceNewtons) {
        rigidBody.applyCentralForce(BulletPhysicsEngine.toVector3f(forceNewtons));
    }

    @Override
    public void applyForceAtPoint(Translation3d forceNewtons, Translation3d pointWorld) {
        // Bullet's applyForce takes a relative offset from center of mass
        Vector3f force = BulletPhysicsEngine.toVector3f(forceNewtons);
        Vector3f comWorld = rigidBody.getPhysicsLocation(null);
        Vector3f relOffset = BulletPhysicsEngine.toVector3f(pointWorld).subtract(comWorld);
        rigidBody.applyForce(force, relOffset);
    }

    @Override
    public void applyTorque(Translation3d torqueNewtonMeters) {
        rigidBody.applyTorque(BulletPhysicsEngine.toVector3f(torqueNewtonMeters));
    }

    @Override
    public double getMassKg() {
        return rigidBody.getMass();
    }

    @Override
    public boolean isStatic() {
        return isStatic;
    }

    @Override
    public Translation3d getLinearVelocityAtPointMPS(Translation3d pointWorld) {
        // v_point = v_com + omega x (point - com)
        Vector3f comWorld = rigidBody.getPhysicsLocation(null);
        Vector3f pointVec = BulletPhysicsEngine.toVector3f(pointWorld);
        Vector3f relOffset = pointVec.subtract(comWorld);

        Vector3f linearVel = rigidBody.getLinearVelocity(null);
        Vector3f angularVel = rigidBody.getAngularVelocity(null);

        // Cross product: omega x r
        Vector3f tangentialVel = angularVel.cross(relOffset);

        // Total velocity at point
        Vector3f totalVel = linearVel.add(tangentialVel);

        return BulletPhysicsEngine.toTranslation3d(totalVel);
    }

    @Override
    public void setUserData(Object data) {
        this.userData = data;
    }

    @Override
    public Object getUserData() {
        return userData;
    }

    @Override
    public void setDamping(double linearDamping, double angularDamping) {
        rigidBody.setDamping((float) linearDamping, (float) angularDamping);
    }

    @Override
    public void setFriction(double friction) {
        rigidBody.setFriction((float) friction);
    }

    @Override
    public void setRestitution(double restitution) {
        rigidBody.setRestitution((float) restitution);
    }

    @Override
    public void setContactReporting(boolean enable) {
        // Bullet default behavior involves collision groups/masks
        // Not implemented in this simplified wrapper yet
    }

    @Override
    public void setCollisionLayer(int layer) {
        // Bullet uses collision groups (bitmasks)
        // Not implemented in this simplified wrapper yet
    }
}
