package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.HighFidelitySwerveSim3DConfig;
import org.ironmaple.simulation.drivesims.latency.LatencyBuffer;
import org.ironmaple.simulation.drivesims.latency.SensorNoiseModel;
import org.ironmaple.simulation.drivesims.tire.PacejkaTireModel;
import org.ironmaple.simulation.drivesims.tire.TireForceResult;
import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 *
 *
 * <h1>High-Fidelity Swerve Drive Simulation in 3D.</h1>
 *
 * <p>Advanced swerve simulation with realistic tire physics for competition-accurate modeling. This simulation runs
 * alongside the standard {@link SwerveDriveSimulation3D} and provides:
 *
 * <ul>
 *   <li>Pacejka "Magic Formula" tire model with non-linear slip curves
 *   <li>Anisotropic friction (different longitudinal vs lateral grip)
 *   <li>Combined slip model using friction ellipse
 *   <li>Scrub torque (steering resistance at low speed)
 *   <li>Self-aligning torque (pneumatic trail effect)
 *   <li>Optional CAN bus latency and sensor noise simulation
 * </ul>
 *
 * <h2>Usage:</h2>
 *
 * <pre>
 * HighFidelitySwerveSim3DConfig hifiConfig = HighFidelitySwerveSim3DConfig.defaultConfig();
 * DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default();
 * HighFidelitySwerveSim3D sim = new HighFidelitySwerveSim3D(driveConfig, hifiConfig, initialPose);
 * sim.registerWithArena(arena, initialPose);
 * </pre>
 *
 * <h2>Reference:</h2>
 *
 * <p>Pacejka, H.B. (2012). "Tire and Vehicle Dynamics" (3rd ed.)
 *
 * @see SwerveDriveSimulation3D for the standard (simpler) 3D swerve simulation
 * @see PacejkaTireModel for the tire physics implementation
 */
public class HighFidelitySwerveSim3D extends AbstractDriveTrainSimulation3D {

    // ==================== Configuration ====================
    private final HighFidelitySwerveSim3DConfig hifiConfig;
    private final PacejkaTireModel tireModel;

    // ==================== Modules ====================
    private final SwerveModuleSimulation[] moduleSimulations;
    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;

    // ==================== Latency Simulation ====================
    @SuppressWarnings("unchecked")
    private final LatencyBuffer<SwerveModuleState>[] commandLatencyBuffers;

    private final SensorNoiseModel[] driveEncoderNoise;
    private final SensorNoiseModel[] steerEncoderNoise;
    private double simulationTimeSeconds = 0.0;

    private Double previousBodyYaw = null;

    // ==================== State Tracking ====================
    private final double[] lastNormalForces;
    private final TireForceResult[] lastTireResults;
    private final double[] lastScrubTorques;
    private final double[] lastSelfAligningTorques;

    // ==================== Suspension Parameters ====================
    private static final double SUSPENSION_STIFFNESS = 12000.0;
    private static final double TARGET_DAMPING_RATIO = 1.2;
    private static final double COMPRESSION_DAMPING_MULT = 1.5;
    private static final double EXPANSION_DAMPING_MULT = 1.2;
    private static final double SUSPENSION_MAX_TRAVEL = 0.1;
    private static final double MAX_SUSPENSION_FORCE = 1000.0;
    private static final double RAYCAST_ORIGIN_OFFSET = 0.5;

    /**
     *
     *
     * <h2>Creates a High-Fidelity Swerve Simulation.</h2>
     *
     * @param driveConfig standard drivetrain configuration (mass, dimensions, modules)
     * @param hifiConfig high-fidelity simulation configuration (Pacejka coefficients, scrub torque, etc.)
     * @param initialPoseOnField initial 2D pose on the field
     */
    @SuppressWarnings("unchecked")
    public HighFidelitySwerveSim3D(
            DriveTrainSimulationConfig driveConfig,
            HighFidelitySwerveSim3DConfig hifiConfig,
            Pose2d initialPoseOnField) {
        super(driveConfig, initialPoseOnField);

        this.hifiConfig = hifiConfig;

        // Create tire model
        this.tireModel = new PacejkaTireModel(
                hifiConfig.longitudinalCoeffs,
                hifiConfig.lateralCoeffs,
                hifiConfig.frictionCoefficientLongitudinal,
                hifiConfig.frictionCoefficientLateral);

        // Initialize modules (reuse existing SwerveModuleSimulation)
        this.moduleTranslations = driveConfig.moduleTranslations;
        this.moduleSimulations = Arrays.stream(driveConfig.swerveModuleSimulationFactories)
                .map(Supplier::get)
                .toArray(SwerveModuleSimulation[]::new);
        this.gyroSimulation = driveConfig.gyroSimulationFactory.get();

        this.kinematics = new SwerveDriveKinematics(moduleTranslations);

        // Initialize tracking arrays
        int numModules = moduleTranslations.length;
        this.lastNormalForces = new double[numModules];
        this.lastTireResults = new TireForceResult[numModules];
        this.lastScrubTorques = new double[numModules];
        this.lastSelfAligningTorques = new double[numModules];
        Arrays.fill(lastTireResults, TireForceResult.zero());

        // Initialize latency buffers if enabled
        if (hifiConfig.enableLatencySimulation) {
            this.commandLatencyBuffers = new LatencyBuffer[numModules];
            this.driveEncoderNoise = new SensorNoiseModel[numModules];
            this.steerEncoderNoise = new SensorNoiseModel[numModules];

            double delaySeconds = hifiConfig.canBusDelay.in(Seconds);
            double posNoiseRad = hifiConfig.encoderPositionNoiseStdDev.in(Radians);
            double velNoiseRadPerSec = hifiConfig.encoderVelocityNoiseStdDev.in(RadiansPerSecond);

            for (int i = 0; i < numModules; i++) {
                // Initialize with neutral state
                SwerveModuleState neutralState = new SwerveModuleState(0, new Rotation2d());
                commandLatencyBuffers[i] = new LatencyBuffer<>(delaySeconds, neutralState);
                driveEncoderNoise[i] = new SensorNoiseModel(posNoiseRad, velNoiseRadPerSec);
                steerEncoderNoise[i] = new SensorNoiseModel(posNoiseRad, velNoiseRadPerSec);
            }
        } else {
            this.commandLatencyBuffers = null;
            this.driveEncoderNoise = null;
            this.steerEncoderNoise = null;
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (physicsBody == null || physicsEngine == null) return;

        // Update simulation time
        simulationTimeSeconds += SimulatedArena.getSimulationDt().in(Seconds);

        // Get current pose and velocities (Direct Access)
        Pose3d pose3d = physicsBody.getPose3d();
        Translation3d linearVel = physicsBody.getLinearVelocityMPS();

        // Update gyro
        updateGyro(pose3d);

        // Process physics (Synchronous only)
        // Process physics (Synchronous only)
        simulateModulesSync(pose3d, linearVel);

        updateMechanisms();
    }

    /**
     *
     *
     * <h2>Synchronous Module Simulation.</h2>
     *
     * <p>Calculates and applies forces directly using Pacejka tire model.
     */
    private void simulateModulesSync(Pose3d pose3d, Translation3d linearVel) {
        Rotation3d rotation = pose3d.getRotation();
        Rotation2d robotHeading = rotation.toRotation2d();

        double robotMassKg = config.robotMass.in(Kilograms);
        double massPerWheel = robotMassKg / moduleTranslations.length;
        double criticalDamping = 2.0 * Math.sqrt(SUSPENSION_STIFFNESS * massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        double groundClearance = config.groundClearance.in(Meters);
        double staticCompression = (massPerWheel * 9.81) / SUSPENSION_STIFFNESS;
        double suspensionRestLength = groundClearance + staticCompression;

        for (int i = 0; i < moduleTranslations.length; i++) {
            Translation2d moduleOffset = moduleTranslations[i];
            SwerveModuleSimulation module = moduleSimulations[i];

            // --- 1. Suspension Calculation ---
            SuspensionResult suspension = calculateSuspension(
                    i, pose3d, rotation, moduleOffset, baseDamping, criticalDamping, suspensionRestLength);
            double normalForceN = suspension.normalForceNewtons;
            lastNormalForces[i] = normalForceN;

            // Apply suspension force
            if (normalForceN > 0) {
                physicsBody.applyForceAtPoint(suspension.suspensionForce, suspension.worldMountPoint);
            }

            // --- 2. Get module state (with latency if enabled) ---
            SwerveModuleState moduleState = getModuleStateWithLatency(i, module);

            // --- 3. Calculate ground velocity at contact point ---
            Translation3d contactVelocity3d = physicsBody.getLinearVelocityAtPointMPS(suspension.worldMountPoint);
            Rotation2d moduleWorldFacing = moduleState.angle.plus(robotHeading);

            // Decompose velocity into longitudinal/lateral in module frame
            double cosAngle = moduleWorldFacing.getCos();
            double sinAngle = moduleWorldFacing.getSin();
            double groundLongVel = contactVelocity3d.getX() * cosAngle + contactVelocity3d.getY() * sinAngle;
            double groundLatVel = -contactVelocity3d.getX() * sinAngle + contactVelocity3d.getY() * cosAngle;

            // --- 4. Calculate slip ---
            double wheelSpeedMPS = moduleState.speedMetersPerSecond;
            double longitudinalSlip = PacejkaTireModel.calculateLongitudinalSlip(wheelSpeedMPS, groundLongVel);
            double slipAngle = PacejkaTireModel.calculateSlipAngle(groundLatVel, Math.abs(groundLongVel));

            // --- 5. Pacejka tire forces ---
            TireForceResult tireResult = tireModel.calculateCombinedForces(longitudinalSlip, slipAngle, normalForceN);
            lastTireResults[i] = tireResult;

            // --- 6. Scrub torque (if enabled and at low speed) ---
            double scrubTorqueNm = 0.0;
            if (hifiConfig.enableScrubTorque) {
                scrubTorqueNm = calculateScrubTorque(i, normalForceN, groundLongVel, module);
            }
            lastScrubTorques[i] = scrubTorqueNm;

            // --- 7. Self-aligning torque (if enabled) ---
            double alignTorqueNm = 0.0;
            if (hifiConfig.enableSelfAligningTorque) {
                alignTorqueNm = calculateSelfAligningTorque(tireResult.lateralForceNewtons(), normalForceN);
            }
            lastSelfAligningTorques[i] = alignTorqueNm;

            // Apply total external torque to steering motor
            module.setSteerExternalTorque(NewtonMeters.of(scrubTorqueNm + alignTorqueNm));

            // --- 8. Convert tire forces to world frame and apply ---
            // Note: Lateral force is negated because Pacejka outputs force in the direction
            // of slip,
            // but we need the reaction force that opposes the lateral sliding motion.
            double fxWorld =
                    tireResult.longitudinalForceNewtons() * cosAngle + tireResult.lateralForceNewtons() * sinAngle;
            double fyWorld =
                    tireResult.longitudinalForceNewtons() * sinAngle - tireResult.lateralForceNewtons() * cosAngle;

            Translation3d tractionForce = new Translation3d(fxWorld, fyWorld, 0);
            physicsBody.applyForceAtPoint(tractionForce, suspension.worldMountPoint);

            // --- 9. Update module simulation ---
            // The module sim still needs to be updated for encoder state tracking
            org.dyn4j.geometry.Vector2 groundVel2d =
                    new org.dyn4j.geometry.Vector2(contactVelocity3d.getX(), contactVelocity3d.getY());
            module.updateSimulationSubTickGetModuleForce(groundVel2d, robotHeading, normalForceN);
        }
    }

    /**
     *
     *
     * <h2>Calculates Suspension Force.</h2>
     */
    private SuspensionResult calculateSuspension(
            int moduleIndex,
            Pose3d pose3d,
            Rotation3d rotation,
            Translation2d moduleOffset,
            double baseDamping,
            double criticalDamping,
            double suspensionRestLength) {

        double chassisHeight = config.chassisHeight.in(Meters);
        double wheelRadius = config.wheelRadius.in(Meters);

        // Correct for COM offset
        double groundClearance = config.groundClearance.in(Meters);
        double mountPointZ = groundClearance - comHeightAboveGround;
        Translation3d localMountPoint = new Translation3d(moduleOffset.getX(), moduleOffset.getY(), mountPointZ);
        Translation3d worldMountPoint = rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

        Translation3d rayDirection = new Translation3d(0, 0, -1);
        Translation3d rayOrigin = worldMountPoint.plus(new Translation3d(0, 0, RAYCAST_ORIGIN_OFFSET));
        double maxRayDistance = suspensionRestLength + 0.1 + wheelRadius + RAYCAST_ORIGIN_OFFSET;

        Optional<PhysicsEngine.RaycastResult> hit =
                physicsEngine.raycast(rayOrigin, rayDirection, maxRayDistance, physicsBody);

        double normalForceNewtons = 0.0;
        Translation3d suspensionForce = Translation3d.kZero;

        if (hit.isPresent()) {
            PhysicsEngine.RaycastResult hitResult = hit.get();
            double groundDistance = hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET;
            double suspensionCompression = suspensionRestLength + wheelRadius - groundDistance;

            if (suspensionCompression > 0) {
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
            }
        }

        return new SuspensionResult(normalForceNewtons, suspensionForce, worldMountPoint);
    }

    /**
     *
     *
     * <h2>Gets Module State with Optional Latency.</h2>
     */
    private SwerveModuleState getModuleStateWithLatency(int moduleIndex, SwerveModuleSimulation module) {
        SwerveModuleState currentState = module.getCurrentState();

        if (!hifiConfig.enableLatencySimulation || commandLatencyBuffers == null) {
            return currentState;
        }

        // Add current state to buffer
        commandLatencyBuffers[moduleIndex].add(simulationTimeSeconds, currentState);

        // Return delayed state
        return commandLatencyBuffers[moduleIndex].get(simulationTimeSeconds);
    }

    /**
     *
     *
     * <h2>Calculates Scrub Torque.</h2>
     *
     * <p>Resistive torque when steering while stationary or slow. Scrub torque = μ × Fz × r_scrub × speedFactor
     *
     * <h3>Reference:</h3>
     *
     * <p>Milliken, W.F. & Milliken, D.L. (1995). "Race Car Vehicle Dynamics" Ch. 5
     *
     * @param moduleIndex module index (0-3)
     * @param normalForceN normal force (Fz)
     * @param groundSpeedMPS ground speed in wheel direction
     * @param module the swerve module simulation
     * @return scrub torque in Newton-meters (opposes steering direction)
     */
    private double calculateScrubTorque(
            int moduleIndex, double normalForceN, double groundSpeedMPS, SwerveModuleSimulation module) {

        double scrubThreshold = hifiConfig.scrubVelocityThreshold.in(MetersPerSecond);

        // Scrub torque diminishes as speed increases (wheel rolls instead of scrubs)
        double speedFactor = Math.max(0, 1.0 - Math.abs(groundSpeedMPS) / scrubThreshold);
        if (speedFactor <= 0) return 0.0;

        // Get steering velocity from module
        double steerVelocityRadPerSec = module.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);

        // Scrub radius = contact patch radius
        double scrubRadius = hifiConfig.contactPatchRadius.in(Meters);

        // Torque = μ × Fz × r_scrub, opposing steering direction
        double maxScrubTorque = hifiConfig.scrubFrictionCoefficient * normalForceN * scrubRadius;

        // Apply as resistive torque opposing steering motion
        double steerVelSign = Math.signum(steerVelocityRadPerSec);
        return -maxScrubTorque * steerVelSign * speedFactor;
    }

    /**
     *
     *
     * <h2>Calculates Self-Aligning Torque.</h2>
     *
     * <p>Torque created by lateral force acting through pneumatic and mechanical trail. T_align = -F_lateral ×
     * (mechanical_trail + pneumatic_trail)
     *
     * <h3>Reference:</h3>
     *
     * <p>Pacejka, H.B. (2012). "Tire and Vehicle Dynamics" Ch. 4
     *
     * @param lateralForceN lateral tire force
     * @param normalForceN normal force (affects pneumatic trail)
     * @return self-aligning torque in Newton-meters
     */
    private double calculateSelfAligningTorque(double lateralForceN, double normalForceN) {
        // Mechanical trail (caster geometry)
        double mechanicalTrail = hifiConfig.mechanicalTrail.in(Meters);

        // Pneumatic trail increases with normal force (contact patch deformation)
        // Simplified model: t_p = coefficient × Fz / 1000
        double pneumaticTrail = hifiConfig.pneumaticTrailCoefficient * normalForceN / 1000.0;

        double totalTrail = mechanicalTrail + pneumaticTrail;

        // Self-aligning torque tries to straighten the wheel
        return -lateralForceN * totalTrail;
    }

    /**
     *
     *
     * <h2>Updates the Gyro Simulation.</h2>
     */
    private void updateGyro(Pose3d pose3d) {
        double currentBodyYaw = pose3d.getRotation().getZ();
        if (previousBodyYaw == null) previousBodyYaw = currentBodyYaw;

        double dYaw = MathUtil.angleModulus(Rotation2d.fromRadians(currentBodyYaw)
                .minus(Rotation2d.fromRadians(previousBodyYaw))
                .getRadians());

        double derivedYawRate = dYaw / SimulatedArena.getSimulationDt().in(Seconds);
        previousBodyYaw = currentBodyYaw;

        gyroSimulation.updateSimulationSubTick(derivedYawRate);
    }

    /**
     *
     *
     * <h2>Rotates a Point by a 3D Rotation.</h2>
     */
    private static Translation3d rotatePoint(Translation3d point, Rotation3d rotation) {
        var quaternion = rotation.getQuaternion();
        double qw = quaternion.getW(), qx = quaternion.getX(), qy = quaternion.getY(), qz = quaternion.getZ();
        double px = point.getX(), py = point.getY(), pz = point.getZ();

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

    // ==================== Accessor Methods ====================

    /**
     *
     *
     * <h2>Gets the Swerve Module Simulations.</h2>
     *
     * @return array of swerve module simulations
     */
    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    /**
     *
     *
     * <h2>Gets the Gyro Simulation.</h2>
     *
     * @return the gyro simulation
     */
    public GyroSimulation getGyroSimulation() {
        return gyroSimulation;
    }

    /**
     *
     *
     * <h2>Gets the Maximum Linear Velocity.</h2>
     *
     * @return the max linear velocity based on module configuration
     */
    public LinearVelocity maxLinearVelocity() {
        return moduleSimulations[0].config.maximumGroundSpeed();
    }

    /**
     *
     *
     * <h2>Gets the Maximum Angular Velocity.</h2>
     *
     * @return the max angular velocity based on drive base radius
     */
    public AngularVelocity maxAngularVelocity() {
        return RadiansPerSecond.of(maxLinearVelocity().in(MetersPerSecond)
                / config.driveBaseRadius().in(Meters));
    }

    /**
     *
     *
     * <h2>Gets the Drive Base Radius.</h2>
     *
     * @return the drive base radius
     */
    public Distance driveBaseRadius() {
        return config.driveBaseRadius();
    }

    /**
     *
     *
     * <h2>Gets the High-Fidelity Configuration.</h2>
     *
     * @return the high-fidelity simulation configuration
     */
    public HighFidelitySwerveSim3DConfig getHiFiConfig() {
        return hifiConfig;
    }

    // ==================== Diagnostic Methods ====================

    /**
     *
     *
     * <h2>Gets Tire Force Result for a Module.</h2>
     *
     * <p>Useful for telemetry and debugging tire behavior.
     *
     * @param moduleIndex module index (0-3)
     * @return the last calculated tire force result
     */
    public TireForceResult getTireResult(int moduleIndex) {
        if (moduleIndex < 0 || moduleIndex >= lastTireResults.length) {
            return TireForceResult.zero();
        }
        return lastTireResults[moduleIndex];
    }

    /**
     *
     *
     * <h2>Gets Normal Force on a Module.</h2>
     *
     * <p>The instantaneous vertical load (Fz) on the wheel. Changes with suspension compression, acceleration, and
     * turning.
     *
     * @param moduleIndex module index (0-3)
     * @return normal force in Newtons
     */
    public double getNormalForce(int moduleIndex) {
        if (moduleIndex < 0 || moduleIndex >= lastNormalForces.length) {
            return 0.0;
        }
        return lastNormalForces[moduleIndex];
    }

    /**
     *
     *
     * <h2>Gets Scrub Torque on a Module.</h2>
     *
     * <p>The resistive torque opposing steering. High at low speed, zero at high speed.
     *
     * @param moduleIndex module index (0-3)
     * @return scrub torque in Newton-meters
     */
    public double getScrubTorque(int moduleIndex) {
        if (moduleIndex < 0 || moduleIndex >= lastScrubTorques.length) {
            return 0.0;
        }
        return lastScrubTorques[moduleIndex];
    }

    /**
     *
     *
     * <h2>Gets Self-Aligning Torque on a Module.</h2>
     *
     * <p>The torque trying to straighten the wheel due to lateral forces acting through the trail.
     *
     * @param moduleIndex module index (0-3)
     * @return self-aligning torque in Newton-meters
     */
    public double getSelfAligningTorque(int moduleIndex) {
        if (moduleIndex < 0 || moduleIndex >= lastSelfAligningTorques.length) {
            return 0.0;
        }
        return lastSelfAligningTorques[moduleIndex];
    }

    /**
     *
     *
     * <h2>Checks if Any Wheel is Sliding.</h2>
     *
     * @return true if any wheel is at the friction limit
     */
    public boolean isAnyWheelSliding() {
        for (TireForceResult result : lastTireResults) {
            if (result.isSliding()) return true;
        }
        return false;
    }

    /**
     *
     *
     * <h2>Gets Average Grip Utilization.</h2>
     *
     * <p>Average of all wheels' grip utilization. 0.0 = no grip used, 1.0 = at limit.
     *
     * @return average grip utilization (0.0 to 1.0+)
     */
    public double getAverageGripUtilization() {
        double sum = 0.0;
        for (TireForceResult result : lastTireResults) {
            sum += result.gripUtilization();
        }
        return sum / lastTireResults.length;
    }

    // ==================== Helper Record ====================

    private record SuspensionResult(
            double normalForceNewtons, Translation3d suspensionForce, Translation3d worldMountPoint) {}
}
