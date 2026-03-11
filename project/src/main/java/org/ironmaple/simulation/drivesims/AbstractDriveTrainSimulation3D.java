package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Represents an Abstract 3D Drivetrain Simulation.</h1>
 *
 * <h3>Simulates the Mass, Collision Space, and Physics of the Drivetrain in 3D.</h3>
 *
 * <p>This class models the physical properties of a drivetrain in a full 3D physics environment, including:
 *
 * <ul>
 *   <li>Mass and inertia tensor
 *   <li>3D collision box
 *   <li>Full 3D forces and torques
 *   <li>Raycast-based ground contact detection
 * </ul>
 *
 * <p>Unlike the 2D {@link AbstractDriveTrainSimulation}, this class does not extend dyn4j Body. Instead, it wraps a
 * {@link PhysicsBody} from the Bullet physics engine.
 */
public abstract class AbstractDriveTrainSimulation3D implements SimulatedArena3D.Simulatable {
    public static final double
            BUMPER_COEFFICIENT_OF_FRICTION = 0.65, // https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction
            BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08; // https://simple.wikipedia.org/wiki/Coefficient_of_restitution

    public final DriveTrainSimulationConfig config;
    protected PhysicsBody physicsBody;
    protected PhysicsEngine physicsEngine;
    protected SimulatedArena3D arena;
    protected final List<MechanismSimulation> mechanisms = new ArrayList<>();

    /**
     *
     *
     * <h2>Creates a 3D Simulation of a Drivetrain.</h2>
     *
     * <p>Sets up the collision space and mass of the chassis in the 3D physics world.
     *
     * @param config the drivetrain configuration
     * @param initialPoseOnField the initial pose (X, Y, heading) - Z is calculated from chassis height
     */
    protected AbstractDriveTrainSimulation3D(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        this.config = config;
    }

    /**
     *
     *
     * <h2>Registers This Drivetrain with a 3D Arena.</h2>
     *
     * <p>Must be called after construction to add the drivetrain to the physics world.
     *
     * @param arena the 3D simulation arena
     * @param initialPose the initial 2D pose (converted to 3D with ground height)
     */
    protected double comHeightAboveGround;

    /**
     *
     *
     * <h2>Gets the Center of Mass Height Above Ground.</h2>
     *
     * @return the absolute Z height of the COM when the robot is at rest
     */
    public double getComHeightAboveGround() {
        return comHeightAboveGround;
    }

    /**
     *
     *
     * <h2>Registers This Drivetrain with a 3D Arena.</h2>
     *
     * <p>Must be called after construction to add the drivetrain to the physics world.
     *
     * @param arena the 3D simulation arena
     * @param initialPose the initial 2D pose (converted to 3D with ground height)
     */
    public void registerWithArena(SimulatedArena3D arena, Pose2d initialPose) {
        this.arena = arena;
        this.physicsEngine = arena.getPhysicsEngine();

        // Create collision shape
        PhysicsShape shape = null;
        if (config.chassisMeshResourcePath.isPresent()) {
            try {
                shape = physicsEngine.createCompoundShapeFromMesh(config.chassisMeshResourcePath.get());
            } catch (Exception e) {
                System.err.println("Failed to load chassis mesh: " + config.chassisMeshResourcePath.get());
                e.printStackTrace();
            }
        }

        double lengthX = config.bumperLengthX.in(Meters);
        double widthY = config.bumperWidthY.in(Meters);
        double chassisBodyHeight = config.chassisHeight.in(Meters);

        // Fallback to box if mesh not loaded
        if (shape == null) {
            Translation3d halfExtents = new Translation3d(lengthX / 2, widthY / 2, chassisBodyHeight / 2);
            shape = physicsEngine.createBoxShape(halfExtents);
        }

        // Initial height of chassis bottom above ground
        double spawnGroundClearance = config.groundClearance.in(Meters);
        // Geometric Center Z (where the box center should be visually)
        double geometricCenterZ = spawnGroundClearance + chassisBodyHeight / 2;

        // Target Center of Mass Z (from config)
        double targetComZ = config.centerOfMass.getZ();
        // If unconfigured (0) or unreasonable, default to geometric center
        if (targetComZ < 0.01 || targetComZ > 1.0) {
            targetComZ = geometricCenterZ;
        }

        // Apply Offset if COM and Geometric Center differ
        double shapeOffsetZ = geometricCenterZ - targetComZ;
        if (Math.abs(shapeOffsetZ) > 0.001) {
            shape = physicsEngine.createOffsetShape(shape, new Translation3d(0, 0, shapeOffsetZ));
        }

        // Store for getSimulatedDriveTrainPose3dGroundRelative
        this.comHeightAboveGround = targetComZ;

        // Create initial 3D pose at COM height
        Pose3d pose3d = new Pose3d(
                new Translation3d(initialPose.getX(), initialPose.getY(), targetComZ),
                new Rotation3d(0, 0, initialPose.getRotation().getRadians()));

        // Create the dynamic body
        double massKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        this.physicsBody = physicsEngine.createDynamicBody(shape, massKg, pose3d);

        // Apply light damping to prevent drift (0.1 is standard value)
        // Bullet damping range is [0, 1] where 1 stops object immediately.
        physicsBody.setDamping(0.1, 0.1);

        // Register as a custom simulation for sub-tick updates
        arena.addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Sets the Robot's Current Pose in the Simulation World.</h2>
     *
     * @param robotPose the desired robot pose (2D)
     */
    public void setSimulationWorldPose(Pose2d robotPose) {
        if (physicsBody == null) return;

        Pose3d currentPose = physicsBody.getPose3d();
        Pose3d newPose = new Pose3d(
                new Translation3d(robotPose.getX(), robotPose.getY(), currentPose.getZ()),
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        physicsBody.setPose3d(newPose);
        physicsBody.setLinearVelocityMPS(new Translation3d());
        physicsBody.setAngularVelocityRadPerSec(new Translation3d());
    }

    /**
     *
     *
     * <h2>Sets the Robot's Speeds.</h2>
     *
     * @param givenSpeeds the desired chassis speeds
     */
    public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        if (physicsBody == null) return;

        // Convert robot-relative speeds to field-relative for the physics body
        Pose3d pose = physicsBody.getPose3d();
        double heading = pose.getRotation().getZ();

        double vx =
                givenSpeeds.vxMetersPerSecond * Math.cos(heading) - givenSpeeds.vyMetersPerSecond * Math.sin(heading);
        double vy =
                givenSpeeds.vxMetersPerSecond * Math.sin(heading) + givenSpeeds.vyMetersPerSecond * Math.cos(heading);

        physicsBody.setLinearVelocityMPS(new Translation3d(vx, vy, 0));
        physicsBody.setAngularVelocityRadPerSec(new Translation3d(0, 0, givenSpeeds.omegaRadiansPerSecond));
    }

    /**
     *
     *
     * <h2>Gets the Actual 3D Pose of the Drivetrain.</h2>
     *
     * <p>Returns the physics body center of mass pose. For visualization where Z=0 is ground, use
     * {@link #getSimulatedDriveTrainPose3dGroundRelative()} instead.
     *
     * @return the 3D pose in the simulation (center of mass)
     */
    public Pose3d getSimulatedDriveTrainPose3d() {
        if (physicsBody == null) return new Pose3d();
        return physicsBody.getPose3d();
    }

    /**
     *
     *
     * <h2>Gets the 3D Pose Adjusted for Ground-Level Visualization.</h2>
     *
     * <p>For AdvantageScope and similar tools where robot models have their origin at the bottom (wheel contact point),
     * this method returns a pose with Z adjusted so Z=0 represents the bottom of the robot at rest on flat ground.
     *
     * @return the 3D pose adjusted for ground-relative visualization
     */
    public Pose3d getSimulatedDriveTrainPose3dGroundRelative() {
        if (physicsBody == null) return new Pose3d();
        Pose3d physicsPose = physicsBody.getPose3d();
        // Subtract half the chassis height to get the bottom of the chassis
        // Z_bottom = Z_center - chassis Height / 2
        double zOffset = config.chassisHeight.in(Meters) / 2;
        return new Pose3d(
                physicsPose.getX(), physicsPose.getY(), physicsPose.getZ() - zOffset, physicsPose.getRotation());
    }

    /**
     *
     *
     * <h2>Gets the Actual 2D Pose of the Drivetrain.</h2>
     *
     * <p>Projects the 3D pose to 2D (ignoring Z and pitch/roll).
     *
     * @return the 2D pose
     */
    public Pose2d getSimulatedDriveTrainPose() {
        Pose3d pose3d = getSimulatedDriveTrainPose3d();
        return new Pose2d(
                pose3d.getX(),
                pose3d.getY(),
                new Rotation2d(pose3d.getRotation().getZ()));
    }

    /**
     *
     *
     * <h2>Gets the Actual Robot-Relative Chassis Speeds.</h2>
     *
     * @return the chassis speeds, robot-relative
     */
    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        ChassisSpeeds fieldRelative = getDriveTrainSimulatedChassisSpeedsFieldRelative();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelative, getSimulatedDriveTrainPose().getRotation());
    }

    /**
     *
     *
     * <h2>Gets the Actual Field-Relative Chassis Speeds.</h2>
     *
     * @return the chassis speeds, field-relative
     */
    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsFieldRelative() {
        if (physicsBody == null) return new ChassisSpeeds();

        Translation3d linearVel = physicsBody.getLinearVelocityMPS();
        Translation3d angularVel = physicsBody.getAngularVelocityRadPerSec();

        return new ChassisSpeeds(linearVel.getX(), linearVel.getY(), angularVel.getZ());
    }

    /**
     *
     *
     * <h2>Gets the Underlying Physics Body.</h2>
     *
     * @return the physics body, or null if not registered
     */
    public PhysicsBody getPhysicsBody() {
        return physicsBody;
    }

    /**
     *
     *
     * <h2>Applies a Force at a Point on the Robot.</h2>
     *
     * @param force the force vector in world coordinates
     * @param pointWorld the application point in world coordinates
     */
    protected void applyForceAtPoint(Translation3d force, Translation3d pointWorld) {
        if (physicsBody != null) {
            physicsBody.applyForceAtPoint(force, pointWorld);
        }
    }

    /**
     *
     *
     * <h2>Applies a Central Force to the Robot.</h2>
     *
     * @param force the force vector in world coordinates
     */
    protected void applyForce(Translation3d force) {
        if (physicsBody != null) {
            physicsBody.applyForce(force);
        }
    }

    /**
     *
     *
     * <h2>Applies a Torque to the Robot.</h2>
     *
     * @param torque the torque vector
     */
    protected void applyTorque(Translation3d torque) {
        if (physicsBody != null) {
            physicsBody.applyTorque(torque);
        }
    }

    /**
     *
     *
     * <h2>Gets the Linear Velocity at a Specific Point.</h2>
     *
     * @param pointWorld the point in world coordinates
     * @return the velocity at that point
     */
    protected Translation3d getLinearVelocityAtPoint(Translation3d pointWorld) {
        if (physicsBody == null) return new Translation3d();
        return physicsBody.getLinearVelocityAtPointMPS(pointWorld);
    }

    /**
     *
     *
     * <h2>Abstract Simulation Sub-Tick Method.</h2>
     *
     * <p>Implemented by subclasses to apply propelling forces.
     */
    @Override
    public abstract void simulationSubTick(int subTickNum);

    /**
     *
     *
     * <h2>Updates the status of registered mechanisms.</h2>
     *
     * <p>Moves the mechanism physics bodies to match the robot's current pose plus the mechanism's relative pose. This
     * should be called in the simulation loop.
     */
    protected void updateMechanisms() {
        if (physicsBody == null) return;
        Pose3d robotPose = physicsBody.getPose3d();

        for (MechanismSimulation mechanism : mechanisms) {
            Pose3d relativePose = mechanism.robotRelativePoseSupplier.get();
            // Transform relative pose to world pose: World = Robot * Relative
            // Note: Pose3d.transformBy puts the argument in the frame of the caller.
            // But here we want to compose: robotPose + relativePose.
            // WPILib's transformBy does: new Pose(this.translation +
            // other.translation.rotateBy(this.rotation), this.rotation + other.rotation)
            // Which is exactly what we want for "robotPose.transformBy(relativePose)" if
            // relativePose is a Transform3d equivalent.
            // But we have Pose3d. Converting Pose3d to Transform3d is trivial.

            Pose3d worldPose = robotPose.transformBy(new edu.wpi.first.math.geometry.Transform3d(
                    relativePose.getTranslation(), relativePose.getRotation()));

            mechanism.body.setPose3d(worldPose);
            mechanism.body.setLinearVelocityMPS(physicsBody.getLinearVelocityMPS());
            mechanism.body.setAngularVelocityRadPerSec(physicsBody.getAngularVelocityRadPerSec());
        }
    }

    /**
     *
     *
     * <h2>Registers a Mechanism with a Collision Mesh.</h2>
     *
     * <p>Creates a kinematic physics body for a mechanism (e.g. elevator, intake) that moves relative to the robot.
     *
     * @param name unique name for the mechanism
     * @param meshResourcePath path to the .obj file (e.g. "meshes/elevator.obj")
     * @param initialRobotRelativePose initial pose of the mechanism relative to the robot center
     * @param robotRelativePoseSupplier supplier that returns the current pose of the mechanism relative to the robot
     */
    public void registerMechanism(
            String name,
            String meshResourcePath,
            Pose3d initialRobotRelativePose,
            Supplier<Pose3d> robotRelativePoseSupplier) {
        if (arena == null) {
            throw new IllegalStateException("Cannot register mechanism before registering drivetrain with arena!");
        }

        try {
            PhysicsShape shape = physicsEngine.createCompoundShapeFromMesh(meshResourcePath);
            if (shape == null) {
                System.err.println("Failed to load mechanism mesh: " + meshResourcePath);
                return;
            }

            // Calculate initial world pose
            Pose3d robotPose = physicsBody != null ? physicsBody.getPose3d() : new Pose3d();
            Pose3d worldPose = robotPose.transformBy(new edu.wpi.first.math.geometry.Transform3d(
                    initialRobotRelativePose.getTranslation(), initialRobotRelativePose.getRotation()));

            // Create kinematic body (moved by code, pushes dynamic bodies)
            PhysicsBody mechBody = physicsEngine.createKinematicBody(shape, worldPose);

            mechanisms.add(new MechanismSimulation(name, mechBody, robotRelativePoseSupplier));

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    protected static class MechanismSimulation {
        public final String name;
        public final PhysicsBody body;
        public final Supplier<Pose3d> robotRelativePoseSupplier;

        public MechanismSimulation(String name, PhysicsBody body, Supplier<Pose3d> robotRelativePoseSupplier) {
            this.name = name;
            this.body = body;
            this.robotRelativePoseSupplier = robotRelativePoseSupplier;
        }
    }
}
