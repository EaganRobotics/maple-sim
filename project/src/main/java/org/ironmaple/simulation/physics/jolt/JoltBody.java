package org.ironmaple.simulation.physics.jolt;

import com.github.stephengold.joltjni.*;
import com.github.stephengold.joltjni.enumerate.EActivation;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.concurrent.atomic.AtomicInteger;
import org.ironmaple.simulation.physics.PhysicsBody;

/**
 *
 *
 * <h1>Jolt Physics Body Implementation</h1>
 *
 * <p>Implements the {@link PhysicsBody} interface using Jolt's Body/BodyId access pattern.
 *
 * <p>Unlike Bullet where you hold a pointer to the body, Jolt uses BodyIds. All body modifications must go through the
 * BodyInterface.
 */
public class JoltBody implements PhysicsBody {
    /** Atomic counter for generating unique body IDs for tracking. */
    private static final AtomicInteger TRACKING_ID_COUNTER = new AtomicInteger(0);

    private final Body body;
    private final PhysicsSystem physicsSystem;
    private final BodyInterface bodyInterface;
    private final boolean isStatic;
    private final int trackingId;
    private final double massKg;
    private Object userData;

    /**
     *
     *
     * <h2>Creates a JoltBody Wrapper.</h2>
     *
     * @param body the Jolt body
     * @param physicsSystem the physics system this body belongs to
     * @param isStatic whether this body is static
     * @param massKg the mass in kilograms
     */
    public JoltBody(Body body, PhysicsSystem physicsSystem, boolean isStatic, double massKg) {
        this.body = body;
        this.physicsSystem = physicsSystem;
        this.bodyInterface = physicsSystem.getBodyInterface();
        this.isStatic = isStatic;
        this.trackingId = TRACKING_ID_COUNTER.incrementAndGet();
        this.massKg = massKg;
    }

    /**
     *
     *
     * <h2>Gets the Jolt Body.</h2>
     *
     * @return the Jolt body
     */
    public Body getBody() {
        return body;
    }

    /**
     *
     *
     * <h2>Gets the Unique Tracking ID.</h2>
     *
     * <p>Used for tracking bodies across thread boundaries.
     *
     * @return the unique tracking ID
     */
    public int getTrackingId() {
        return trackingId;
    }

    /**
     *
     *
     * <h2>Gets the Jolt Internal Body ID.</h2>
     *
     * <p>Used for raycast hit comparison with Jolt's internal body IDs.
     *
     * @return the Jolt body ID as an int
     */
    public int getJoltBodyId() {
        return body.getId();
    }

    /**
     *
     *
     * <h2>Gets the Physics System.</h2>
     *
     * @return the physics system
     */
    public PhysicsSystem getPhysicsSystem() {
        return physicsSystem;
    }

    @Override
    public Pose3d getPose3d() {
        RVec3 position = bodyInterface.getPosition(body.getId());
        Quat rotation = bodyInterface.getRotation(body.getId());
        Pose3d pose = toPose3d(position, rotation);
        // DEBUG: Log pose reading occasionally
        if (Math.random() < 0.01) {
            // System.out.println("[JoltBody] getPose3d id=" + trackingId + " pos=(" +
            // pose.getX() + ", " + pose.getY()
            // + ", " + pose.getZ() + ")");
        }
        return pose;
    }

    @Override
    public void setPose3d(Pose3d pose) {
        RVec3 position = toRVec3(pose.getTranslation());
        Quat rotation = toQuat(pose.getRotation());
        bodyInterface.setPositionAndRotation(body.getId(), position, rotation, EActivation.Activate);
    }

    @Override
    public Translation3d getLinearVelocityMPS() {
        Vec3 velocity = bodyInterface.getLinearVelocity(body.getId());
        return toTranslation3d(velocity);
    }

    @Override
    public void setLinearVelocityMPS(Translation3d velocityMPS) {
        Vec3 velocity = toVec3(velocityMPS);
        bodyInterface.setLinearVelocity(body.getId(), velocity);
    }

    @Override
    public Translation3d getAngularVelocityRadPerSec() {
        Vec3 angVel = bodyInterface.getAngularVelocity(body.getId());
        return toTranslation3d(angVel);
    }

    @Override
    public void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec) {
        Vec3 angVel = toVec3(angularVelocityRadPerSec);
        bodyInterface.setAngularVelocity(body.getId(), angVel);
    }

    @Override
    public void applyForce(Translation3d force) {
        if (bodyInterface != null) {
            bodyInterface.activateBody(body.getId());
            bodyInterface.addForce(body.getId(), toVec3(force));
            // DEBUG: Log force application
            if (Math.random() < 0.05) {
                JoltPhysicsEngine.log("FORCE", "Id=" + trackingId + " Force=" + force);
            }
        }
    }

    @Override
    public void applyForceAtPoint(Translation3d force, Translation3d point) {
        if (bodyInterface != null) {
            bodyInterface.activateBody(body.getId());
            bodyInterface.addForce(body.getId(), toVec3(force), toRVec3(point));
            // DEBUG: Log force application
            if (Math.random() < 0.05) {
                JoltPhysicsEngine.log("FORCE_AT_POINT", "Id=" + trackingId + " Force=" + force + " Point=" + point);
            }
        }
    }

    @Override
    public void applyTorque(Translation3d torqueNewtonMeters) {
        Vec3 torque = toVec3(torqueNewtonMeters);
        bodyInterface.activateBody(body.getId());
        bodyInterface.addTorque(body.getId(), torque);
        // DEBUG
        double mag = Math.sqrt(torqueNewtonMeters.getX() * torqueNewtonMeters.getX()
                + torqueNewtonMeters.getY() * torqueNewtonMeters.getY()
                + torqueNewtonMeters.getZ() * torqueNewtonMeters.getZ());
        if (mag > 0.1) {
            // System.out.println("[JoltBody] applyTorque id=" + trackingId + " torque=" +
            // mag + "Nm");
        }
    }

    @Override
    public double getMassKg() {
        return massKg;
    }

    @Override
    public boolean isStatic() {
        return isStatic;
    }

    @Override
    public Translation3d getLinearVelocityAtPointMPS(Translation3d pointWorld) {
        // v_point = v_com + omega x (point - com)
        RVec3 centerOfMass = bodyInterface.getCenterOfMassPosition(body.getId());
        Vec3 linearVel = bodyInterface.getLinearVelocity(body.getId());
        Vec3 angularVel = bodyInterface.getAngularVelocity(body.getId());

        // Calculate relative offset
        double rx = pointWorld.getX() - centerOfMass.xx();
        double ry = pointWorld.getY() - centerOfMass.yy();
        double rz = pointWorld.getZ() - centerOfMass.zz();

        // Cross product: omega x r
        double tx = angularVel.getY() * rz - angularVel.getZ() * ry;
        double ty = angularVel.getZ() * rx - angularVel.getX() * rz;
        double tz = angularVel.getX() * ry - angularVel.getY() * rx;

        // Total velocity
        return new Translation3d(linearVel.getX() + tx, linearVel.getY() + ty, linearVel.getZ() + tz);
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
        // Jolt uses motion properties for damping
        if (!body.isStatic()) {
            MotionProperties mp = body.getMotionProperties();
            mp.setLinearDamping((float) linearDamping);
            mp.setAngularDamping((float) angularDamping);
        }
    }

    @Override
    public int getBodyId() {
        return trackingId;
    }

    @Override
    public void setFriction(double friction) {
        bodyInterface.setFriction(body.getId(), (float) friction);
    }

    @Override
    public void setRestitution(double restitution) {
        bodyInterface.setRestitution(body.getId(), (float) restitution);
    }

    @Override
    public void setContactReporting(boolean enable) {
        // Not currently supported by Jolt wrapper in this binding, but we can store it
        // or
        // map it to a layer change if needed.
        // For now, it's a no-op until we implement ContactContactListener.
    }

    @Override
    public void setCollisionLayer(int layer) {
        bodyInterface.setObjectLayer(body.getId(), layer);
    }

    // ========== Conversion Utilities ==========

    /** Converts WPILib Translation3d to Jolt Vec3. */
    public static Vec3 toVec3(Translation3d t) {
        return new Vec3((float) t.getX(), (float) t.getY(), (float) t.getZ());
    }

    /** Converts WPILib Translation3d to Jolt RVec3. */
    public static RVec3 toRVec3(Translation3d t) {
        return new RVec3(t.getX(), t.getY(), t.getZ());
    }

    /** Converts Jolt Vec3 to WPILib Translation3d. */
    public static Translation3d toTranslation3d(Vec3 v) {
        return new Translation3d(v.getX(), v.getY(), v.getZ());
    }

    /** Converts Jolt RVec3 to WPILib Translation3d. */
    public static Translation3d toTranslation3d(RVec3 v) {
        return new Translation3d(v.xx(), v.yy(), v.zz());
    }

    /** Converts WPILib Rotation3d to Jolt Quat. */
    public static Quat toQuat(Rotation3d r) {
        edu.wpi.first.math.geometry.Quaternion q = r.getQuaternion();
        return new Quat((float) q.getX(), (float) q.getY(), (float) q.getZ(), (float) q.getW());
    }

    /** Converts Jolt Quat to WPILib Rotation3d. */
    public static Rotation3d toRotation3d(Quat q) {
        return new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(q.getW(), q.getX(), q.getY(), q.getZ()));
    }

    /** Converts Jolt position and rotation to WPILib Pose3d. */
    public static Pose3d toPose3d(RVec3 position, Quat rotation) {
        return new Pose3d(toTranslation3d(position), toRotation3d(rotation));
    }
}
