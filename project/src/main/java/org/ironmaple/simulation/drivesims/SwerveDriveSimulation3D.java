package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.debugging.SimDebugLogger;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 *
 *
 * <h1>Simulates a Swerve Drivetrain in 3D.</h1>
 *
 * <p>This class provides high-fidelity 3D simulation of a swerve drivetrain with:
 *
 * <ul>
 *   <li>Raycast-based wheel contact detection for proper ground interaction
 *   <li>3D friction forces calculated relative to ground surface normals
 *   <li>Support for driving on inclined surfaces (ramps, charge stations)
 *   <li>Realistic wheel slip and traction limiting
 * </ul>
 */
public class SwerveDriveSimulation3D extends AbstractDriveTrainSimulation3D {
    private final SwerveModuleSimulation[] moduleSimulations;
    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;

    private Double previousBodyYaw = null;

    private int debugLogCounter = 0;

    // ==================== Suspension Parameters ====================

    /** Suspension spring constant (N/m). Lower = softer, less bouncy. Higher = rigid FRC frame. */
    private static final double SUSPENSION_STIFFNESS = 12000.0;

    /** Target damping ratio. 1.0 = critically damped (no bounce). >1.0 = overdamped. */
    private static final double TARGET_DAMPING_RATIO = 1.2;

    /** Compression damping multiplier (full damping on bump). */
    private static final double COMPRESSION_DAMPING_MULT = 1.5;

    /** Expansion damping multiplier (more damping on rebound to prevent bounce-back). */
    private static final double EXPANSION_DAMPING_MULT = 1.2;

    /** Maximum suspension compression/extension (meters). */
    private static final double SUSPENSION_MAX_TRAVEL = 0.1;

    /** Maximum force per wheel to prevent physics explosions (N). */
    private static final double MAX_SUSPENSION_FORCE = 1000.0; // 4 wheels * 1000N = 4000N = 400kg support

    /**
     *
     *
     * <h2>Creates a 3D Swerve Drive Simulation.</h2>
     *
     * @param config the drivetrain configuration
     * @param initialPoseOnField the initial 2D pose
     */
    public SwerveDriveSimulation3D(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        super(config, initialPoseOnField);

        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = Arrays.stream(config.swerveModuleSimulationFactories)
                .map(Supplier::get)
                .toArray(SwerveModuleSimulation[]::new);
        this.gyroSimulation = config.gyroSimulationFactory.get();

        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (physicsBody == null || physicsEngine == null) return;

        SimDebugLogger.incrementTick();

        // Get body ID - works for all PhysicsBody implementations
        int bodyId = physicsBody.getBodyId();

        // Get current pose and velocities (Sync mode: direct physics access)
        Pose3d pose3d = physicsBody.getPose3d();
        Translation3d linearVel = physicsBody.getLinearVelocityMPS();

        // Handle body wrapper types for angular velocity
        double yawRate = physicsBody.getAngularVelocityRadPerSec().getZ();

        var rotation = pose3d.getRotation();
        Rotation2d heading2d = rotation.toRotation2d();

        // Log every 50 ticks to avoid spam
        if (subTickNum == 0) {
            SimDebugLogger.logPose(String.format(
                    "X=%.3f Y=%.3f Z=%.3f Yaw=%.1f° Roll=%.1f° Pitch=%.1f°",
                    pose3d.getX(),
                    pose3d.getY(),
                    pose3d.getZ(),
                    Math.toDegrees(heading2d.getRadians()),
                    Math.toDegrees(rotation.getX()),
                    Math.toDegrees(rotation.getY())));

            SimDebugLogger.logVelocity(String.format(
                    "Linear: vx=%.3f vy=%.3f vz=%.3f | YawRate=%.3f rad/s",
                    linearVel.getX(), linearVel.getY(), linearVel.getZ(), yawRate));

            SimDebugLogger.logHeading(
                    String.format("Heading2d=%.1f° (from toRotation2d)", Math.toDegrees(heading2d.getRadians())));
        }

        // Apply suspension and traction forces (Synchronous calculation)
        simulateModulesSync(pose3d, linearVel, bodyId);

        // Calculate Yaw Rate from Pose Delta to ensure robustness against physics
        // engine quirks
        double currentBodyYaw = pose3d.getRotation().getZ();
        if (previousBodyYaw == null) previousBodyYaw = currentBodyYaw;

        double dYaw = edu.wpi.first.math.MathUtil.angleModulus(Rotation2d.fromRadians(currentBodyYaw)
                .minus(Rotation2d.fromRadians(previousBodyYaw))
                .getRadians());

        double derivedYawRate =
                dYaw / org.ironmaple.simulation.SimulatedArena.getSimulationDt().in(Seconds);
        previousBodyYaw = currentBodyYaw;

        // Update gyro with derived yaw rate
        // Update gyro with derived yaw rate
        gyroSimulation.updateSimulationSubTick(derivedYawRate);

        // Update mechanisms
        updateMechanisms();
    }

    /** Calculates the critical damping coefficient for a given mass per wheel. c_crit = 2 * sqrt(k * m) */
    private double calculateCriticalDamping(double massPerWheelKg) {
        return 2.0 * Math.sqrt(SUSPENSION_STIFFNESS * massPerWheelKg);
    }

    /**
     *
     *
     * <h2>Rotates a Point by a 3D Rotation.</h2>
     */
    private Translation3d rotatePoint(Translation3d point, edu.wpi.first.math.geometry.Rotation3d rotation) {
        // Use quaternion rotation
        var quaternion = rotation.getQuaternion();
        double qw = quaternion.getW(), qx = quaternion.getX(), qy = quaternion.getY(), qz = quaternion.getZ();
        double px = point.getX(), py = point.getY(), pz = point.getZ();

        // Quaternion rotation: q * p * q^-1
        double rx = qw * qw * px
                + 2 * qy * qw * pz
                - 2 * qz * qw * py
                + qx * qx * px
                + 2 * qy * qx * py
                + 2 * qz * qx * pz
                - qz * qz * px
                - qy * qy * px;
        double ry = 2 * qx * qy * px
                + qy * qy * py
                + 2 * qz * qy * pz
                + 2 * qw * qz * px
                - qz * qz * py
                + qw * qw * py
                - 2 * qx * qw * pz
                - qx * qx * py;
        double rz = 2 * qx * qz * px
                + 2 * qy * qz * py
                + qz * qz * pz
                - 2 * qw * qy * px
                - qy * qy * pz
                + 2 * qw * qx * py
                - qx * qx * pz
                + qw * qw * pz;

        return new Translation3d(rx, ry, rz);
    }

    /**
     *
     *
     * <h2>Gets the Maximum Linear Velocity.</h2>
     */
    public LinearVelocity maxLinearVelocity() {
        return moduleSimulations[0].config.maximumGroundSpeed();
    }

    /**
     *
     *
     * <h2>Gets the Maximum Angular Velocity.</h2>
     */
    public AngularVelocity maxAngularVelocity() {
        return RadiansPerSecond.of(maxLinearVelocity().in(MetersPerSecond)
                / config.driveBaseRadius().in(Meters));
    }

    /**
     *
     *
     * <h2>Gets the 3D Pose Adjusted for Ground-Level Visualization.</h2>
     *
     * <p>Overridden to account for suspension rest length and wheel radius.
     *
     * @return the 3D pose such that Z=0 corresponds to the bottom of the wheels at rest
     */
    @Override
    public Pose3d getSimulatedDriveTrainPose3dGroundRelative() {
        if (physicsBody == null) return new Pose3d();
        Pose3d physicsPose = physicsBody.getPose3d();

        // Calculate offset from center of mass to bottom of wheels at rest
        // Uses configured values
        // Z-offset: Calculate ground pose by subtracting COM height
        // Physics Body Origin is at (comHeightAboveGround)
        // Ground is at 0
        // So offset = comHeightAboveGround
        double zOffset = comHeightAboveGround;

        // X/Y-offset: transform world CoM pose to world Geometric Center pose
        // We need to subtract the rotated CoM X/Y offset from the physics body position
        Translation2d comXYOffset = config.centerOfMass.toTranslation2d();
        Translation2d rotatedXYOffset =
                comXYOffset.rotateBy(physicsPose.getRotation().toRotation2d());

        return new Pose3d(
                physicsPose.getX() - rotatedXYOffset.getX(),
                physicsPose.getY() - rotatedXYOffset.getY(),
                physicsPose.getZ() - zOffset,
                physicsPose.getRotation());
    }

    /**
     * for (int i = 0; i < moduleSimulations.length; i++) { double wheelTorque =
     * moduleSimulations[i].getDriveWheelTorque(); driveForces[i] = wheelTorque / config.wheelRadius.in(Meters); }
     * proxy.queueSwerveInput(capturedStates, driveForces); } }
     *
     * <p>/**
     *
     * <h2>Simulates All Swerve Modules (Suspension + Traction) in Sync Mode.</h2>
     *
     * <p>Iterates through each module to:
     *
     * <ul>
     *   <li>Calculate suspension force (raycast)
     *   <li>Calculate ground velocity
     *   <li>Update module simulation with dynamic normal force
     *   <li>Apply resulting suspension and traction forces to the body
     * </ul>
     */
    private void simulateModulesSync(Pose3d pose3d, Translation3d linearVel, int bodyId) {
        var rotation = pose3d.getRotation();
        Rotation2d robotHeading = rotation.toRotation2d();
        boolean isThreaded = false; // Sync mode by definition

        // Calculate per-wheel mass for damping
        double robotMassKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        double massPerWheel = robotMassKg / moduleTranslations.length;
        double criticalDamping = calculateCriticalDamping(massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        // Dynamic suspension rest length calculation
        double groundClearance = config.groundClearance.in(Meters);
        double staticCompression = (massPerWheel * 9.81) / SUSPENSION_STIFFNESS;
        double suspensionRestLength = groundClearance + staticCompression;

        double wheelRadius = config.wheelRadius.in(Meters);

        for (int i = 0; i < moduleTranslations.length; i++) {
            Translation2d moduleOffset = moduleTranslations[i];
            SwerveModuleSimulation module = moduleSimulations[i];

            // --- 1. Suspension Calculation ---
            // Local mount point Z is relative to the Physics Body Origin (COM)
            // Geometric bottom of chassis is at (groundClearance)
            // COM is at (comHeightAboveGround)
            // MountPoint (Wheel Center) is at (groundClearance + wheelRadius)
            // So mount point in body frame is (groundClearance + wheelRadius -
            // comHeightAboveGround)
            double mountPointZ = groundClearance + wheelRadius - comHeightAboveGround;
            Translation3d localMountPoint = new Translation3d(moduleOffset.getX(), moduleOffset.getY(), mountPointZ);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            Translation3d rayDirection = new Translation3d(0, 0, -1);
            // Start raycast from FIXED height above expected ground level
            double fixedRayStartHeight = 0.5;
            Translation3d rayOrigin =
                    new Translation3d(worldMountPoint.getX(), worldMountPoint.getY(), fixedRayStartHeight);
            double maxRayDistance = fixedRayStartHeight + suspensionRestLength + SUSPENSION_MAX_TRAVEL + wheelRadius;

            // Log occasionally
            // if (i == 0 && Math.random() < 0.01) {
            // System.out.println("Suspension: mountZ=" + mountPointZ + " poseZ=" +
            // pose3d.getZ() + " grndClear="
            // + groundClearance + " comH=" + comHeightAboveGround);
            // }

            // Get raycast result (Sync mode: direct)
            PhysicsEngine.RaycastResult hitResult = null;

            Optional<PhysicsEngine.RaycastResult> hit =
                    physicsEngine.raycast(rayOrigin, rayDirection, maxRayDistance, physicsBody);
            hitResult = hit.orElse(null);

            double normalForceNewtons = 0.0;
            Translation3d suspensionForce = Translation3d.kZero;

            if (hitResult != null) {
                // Calculate distance from wheel center to ground
                double groundHeight = fixedRayStartHeight - hitResult.hitDistance();
                double wheelCenterHeight = worldMountPoint.getZ();
                double groundDistance = wheelCenterHeight - groundHeight;
                double suspensionCompression = suspensionRestLength + wheelRadius - groundDistance;

                if (suspensionCompression > 0) {
                    // Get velocity of mount point
                    Translation3d mountVelocity = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
                    double compressionVelocity = -mountVelocity.getZ();

                    double dampingCoeff = compressionVelocity > 0
                            ? baseDamping * COMPRESSION_DAMPING_MULT
                            : baseDamping * EXPANSION_DAMPING_MULT;

                    double springForce = SUSPENSION_STIFFNESS * suspensionCompression;
                    double damperForce = dampingCoeff * compressionVelocity;

                    normalForceNewtons = Math.max(0, springForce + damperForce);
                    normalForceNewtons = Math.min(normalForceNewtons, MAX_SUSPENSION_FORCE);

                    Translation3d normal = hitResult.hitNormal();
                    suspensionForce = normal.times(normalForceNewtons);

                    // Debug logging for bounciness investigation (rate limited to ~4Hz assuming
                    // 100Hz physics)
                    // if (i == 0 && debugLogCounter++ % 25 == 0) {
                    // System.out.printf(
                    // "[SuspensionDebug] Mode:Sync Z:%.4f Vz:%.4f Comp:%.4f Norm:%.2f Spring:%.2f
                    // Damp:%.2f%n",
                    // pose3d.getZ(),
                    // mountVelocity.getZ(),
                    // suspensionCompression,
                    // normalForceNewtons,
                    // springForce,
                    // damperForce);
                    // }
                }
            }

            // --- 2. Traction/Propulsion Calculation ---
            Translation3d pointVelocity3d = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
            org.dyn4j.geometry.Vector2 groundVelocity2d =
                    new org.dyn4j.geometry.Vector2(pointVelocity3d.getX(), pointVelocity3d.getY());

            org.dyn4j.geometry.Vector2 tractionForce2d =
                    module.updateSimulationSubTickGetModuleForce(groundVelocity2d, robotHeading, normalForceNewtons);

            Translation3d tractionForce3d = new Translation3d(tractionForce2d.x, tractionForce2d.y, 0);

            // Apply forces
            if (normalForceNewtons > 0) {
                physicsBody.applyForceAtPoint(suspensionForce, worldMountPoint);
            }
            physicsBody.applyForceAtPoint(tractionForce3d, worldMountPoint);

            // Debug logging (only module 0)
            if (i == 0) {
                SimDebugLogger.logForce(String.format(
                        "Mod0: suspension=%.1fN traction=(%.1f,%.1f)N groundVel=(%.3f,%.3f)m/s heading=%.1f° threaded=%b",
                        normalForceNewtons,
                        tractionForce2d.x,
                        tractionForce2d.y,
                        groundVelocity2d.x,
                        groundVelocity2d.y,
                        Math.toDegrees(robotHeading.getRadians()),
                        isThreaded));
            }
        }
    }

    /**
     *
     *
     * <h2>Gets the Swerve Module Simulations.</h2>
     */
    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    /**
     *
     *
     * <h2>Gets the Gyro Simulation.</h2>
     */
    public GyroSimulation getGyroSimulation() {
        return gyroSimulation;
    }

    /**
     *
     *
     * <h2>Gets the Drive Base Radius.</h2>
     */
    public Distance driveBaseRadius() {
        return config.driveBaseRadius();
    }
}
