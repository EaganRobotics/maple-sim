package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation3D;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation3D;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/**
 *
 *
 * <h2>Simulates a Shooter Mechanism in 3D.</h2>
 *
 * <p>Models flywheel physics, inputs from feed sources (like intakes), and launching of projectiles.
 */
public class ShooterSimulation3D implements SimulatedArena3D.Simulatable {
    private final GamePieceInfo gamePieceInfo;
    private final AbstractDriveTrainSimulation3D driveTrainSimulation;
    private final Supplier<Pose3d> shooterPoseSupplier;
    private final List<MapleMotorSim> flywheelMotorSims = new ArrayList<>();

    private IntakeSimulation3D intakeSource = null;
    private int gamePiecesLoaded = 0;
    private int capacity = 1;

    private Voltage appliedVoltage = Volts.zero();
    private Consumer<GamePieceOnFieldSimulation3D> onShootCallback = (p) -> {};

    /**
     * Start building a Shooter Simulation.
     *
     * @param gamePieceInfo the type of game piece this shooter handles
     * @return a new Builder instance
     */
    public static Builder create(GamePieceInfo gamePieceInfo) {
        return new Builder(gamePieceInfo);
    }

    public static class Builder {
        private final GamePieceInfo gamePieceInfo;
        private AbstractDriveTrainSimulation3D driveTrainSimulation;
        private Supplier<Pose3d> shooterPoseSupplier;
        private int capacity = 1;
        private final List<SimMotorConfigs> motors = new ArrayList<>();
        private IntakeSimulation3D intakeSource = null;

        public Builder(GamePieceInfo gamePieceInfo) {
            this.gamePieceInfo = gamePieceInfo;
        }

        public Builder onRobot(AbstractDriveTrainSimulation3D drive, Supplier<Pose3d> poseSupplier) {
            this.driveTrainSimulation = drive;
            this.shooterPoseSupplier = poseSupplier;
            return this;
        }

        public Builder withMotor(SimMotorConfigs config) {
            this.motors.add(config);
            return this;
        }

        public Builder withCapacity(int capacity) {
            this.capacity = capacity;
            return this;
        }

        public Builder withIntakeSource(IntakeSimulation3D intake) {
            this.intakeSource = intake;
            return this;
        }

        public ShooterSimulation3D build() {
            if (driveTrainSimulation == null || shooterPoseSupplier == null) {
                throw new IllegalStateException("Drive simulation and shooter pose supplier must be set via onRobot()");
            }
            ShooterSimulation3D sim = new ShooterSimulation3D(
                    gamePieceInfo, driveTrainSimulation, shooterPoseSupplier, motors.toArray(new SimMotorConfigs[0]));
            sim.withCapacity(capacity);
            if (intakeSource != null) sim.withIntakeSource(intakeSource);
            return sim;
        }
    }

    private ShooterSimulation3D(
            GamePieceInfo gamePieceInfo,
            AbstractDriveTrainSimulation3D driveTrainSimulation,
            Supplier<Pose3d> shooterPoseSupplier,
            SimMotorConfigs... flywheelMotors) {
        this.gamePieceInfo = gamePieceInfo;
        this.driveTrainSimulation = driveTrainSimulation;
        this.shooterPoseSupplier = shooterPoseSupplier;

        for (SimMotorConfigs config : flywheelMotors) {
            MapleMotorSim sim = new MapleMotorSim(config);
            // Sets a dynamic controller that reads the current applied voltage
            sim.useMotorController((mechAngle, mechVel, encAngle, encVel) -> appliedVoltage);
            this.flywheelMotorSims.add(sim);
        }

        // Automatically register with the 3D arena
        SimulatedArena3D.getInstance().addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Pairs this shooter with an Intake.</h2>
     *
     * <p>The shooter will automatically poll the intake for game pieces during the simulation loop. If the intake has a
     * game piece and the shooter has space, it will be transferred.
     *
     * @param intake the intake simulation to pull from
     * @return this instance
     */
    public ShooterSimulation3D withIntakeSource(IntakeSimulation3D intake) {
        this.intakeSource = intake;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the storage capacity (ammo limit).</h2>
     *
     * @param capacity max game pieces
     * @return this instance
     */
    public ShooterSimulation3D withCapacity(int capacity) {
        this.capacity = capacity;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the voltage for the flywheel motors.</h2>
     *
     * @param voltage target voltage
     */
    public void setShooterVoltage(Voltage voltage) {
        this.appliedVoltage = voltage;
    }

    /**
     *
     *
     * <h2>Manually adds game pieces to the shooter.</h2>
     *
     * @param amount number to add
     */
    public void addGamePiece(int amount) {
        this.gamePiecesLoaded = Math.min(this.gamePiecesLoaded + amount, capacity);
    }

    /**
     *
     *
     * <h2>Checks if a game piece is loaded.</h2>
     *
     * @return true if count > 0
     */
    public boolean hasGamePiece() {
        return gamePiecesLoaded > 0;
    }

    public int getGamePiecesLoaded() {
        return gamePiecesLoaded;
    }

    /**
     *
     *
     * <h2>Gets the average angular velocity of the flywheels.</h2>
     *
     * @return average velocity
     */
    public AngularVelocity getVelocity() {
        if (flywheelMotorSims.isEmpty()) return RadiansPerSecond.zero();
        AngularVelocity total = RadiansPerSecond.zero();
        for (MapleMotorSim sim : flywheelMotorSims) {
            total = total.plus(sim.getVelocity());
        }
        return total.div(flywheelMotorSims.size());
    }

    public Current getStatorCurrent() {
        Current total = Amps.zero();
        for (MapleMotorSim sim : flywheelMotorSims) {
            total = total.plus(sim.getStatorCurrent());
        }
        return total;
    }

    /**
     *
     *
     * <h2>Shoots a game piece.</h2>
     *
     * <p>Creates a physics-enabled game piece with initial velocity. Uses full Bullet 3D physics for realistic
     * trajectory, wall collisions, and bouncing.
     *
     * @param velocity the muzzle velocity of the game piece
     * @param pitch the launch angle (up/down) relative to the shooter's horizontal
     */
    public void shoot(LinearVelocity velocity, Angle pitch) {
        if (gamePiecesLoaded <= 0) return;

        gamePiecesLoaded--;

        // 1. Calculate Shooter Pose in World Frame
        Pose3d robotPose = driveTrainSimulation.getSimulatedDriveTrainPose3d();
        Pose3d shooterPoseRelative = shooterPoseSupplier.get();
        Transform3d robotToShooter =
                new Transform3d(shooterPoseRelative.getTranslation(), shooterPoseRelative.getRotation());
        Pose3d shooterPoseWorld = robotPose.plus(robotToShooter);

        // 2. Calculate Muzzle Velocity in World Frame
        double vMuzzle = velocity.in(edu.wpi.first.units.Units.MetersPerSecond);
        double pitchRad = pitch.in(Radians);

        // Velocity components in shooter's local frame (X = forward, Z = up)
        double vXLocal = vMuzzle * Math.cos(pitchRad);
        double vZLocal = vMuzzle * Math.sin(pitchRad);

        // Rotate by shooter's yaw to get world XY velocity
        Rotation2d shooterYaw = shooterPoseWorld.getRotation().toRotation2d();
        Translation2d vXYWorldFromMuzzle = new Translation2d(vXLocal, 0).rotateBy(shooterYaw);

        // Add robot velocity
        ChassisSpeeds robotSpeeds = driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d robotVXY = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        Translation2d totalVXY = vXYWorldFromMuzzle.plus(robotVXY);

        Translation3d initialVelocity = new Translation3d(totalVXY.getX(), totalVXY.getY(), vZLocal);

        // 3. Create physics-enabled game piece (uses Bullet 3D physics for real
        // collisions)
        SimulatedArena3D arena = SimulatedArena3D.getInstance();
        // Create sphere shape for fuel ball (approximate radius)
        double fuelRadius = 0.05; // 5cm radius for fuel ball (TODO: get from gamePieceInfo)
        var shape = arena.getPhysicsEngine().createSphereShape(fuelRadius);

        var info3d = new GamePieceOnFieldSimulation3D.GamePieceInfo3D(
                gamePieceInfo.type(),
                shape,
                gamePieceInfo.gamePieceMass(),
                0.8, // friction
                0.1, // linearDamping
                0.1, // angularDamping
                0.5 // restitution (bounciness)
                );

        GamePieceOnFieldSimulation3D projectile = new GamePieceOnFieldSimulation3D(arena, info3d, shooterPoseWorld);

        // Set initial velocity for the projectile
        projectile.getPhysicsBody().setLinearVelocityMPS(initialVelocity);

        System.out.printf(
                "[ShooterSimulation3D] Shot game piece! Pos=(%.2f,%.2f,%.2f) Vel=(%.2f,%.2f,%.2f) m/s%n",
                shooterPoseWorld.getX(),
                shooterPoseWorld.getY(),
                shooterPoseWorld.getZ(),
                initialVelocity.getX(),
                initialVelocity.getY(),
                initialVelocity.getZ());
    }

    /**
     *
     *
     * <h2>Registers a callback for when a shot is fired.</h2>
     *
     * @param callback consumer that receives the fired game piece
     * @return this instance
     */
    public ShooterSimulation3D onShoot(Consumer<GamePieceOnFieldSimulation3D> callback) {
        this.onShootCallback = callback;
        return this;
    }

    /**
     *
     *
     * <h2>Supplies poses for game pieces currently loaded in this shooter.</h2>
     *
     * <p>Use this method to visualize pieces inside the robot in AdvantageScope.
     *
     * @param type the game piece type to query
     * @param poseList the list to add poses to
     */
    public void supplyRobotPoses(String type, List<Pose3d> poseList) {
        if (!type.equals(gamePieceInfo.type())) return;
        Pose3d robotPose = driveTrainSimulation.getSimulatedDriveTrainPose3d();
        Pose3d shooterPoseRelative = shooterPoseSupplier.get();
        for (int i = 0; i < gamePiecesLoaded; i++) {
            Transform3d offset = new Transform3d(
                    shooterPoseRelative.getTranslation().plus(new Translation3d(0, 0, i * 0.05)),
                    shooterPoseRelative.getRotation());
            poseList.add(robotPose.plus(offset));
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        // Update motors
        for (MapleMotorSim sim : flywheelMotorSims) {
            sim.update(SimulatedArena3D.getSimulationDt());
        }

        // Poll intake
        // Currently IntakeSimulation3D does not have a "obtainGamePieceFromIntake"
        // method returning boolean like the 2D
        // one.
        // We need to check if IntakeSimulation3D has a way to get pieces.
        // Looking at IntakeSimulation3D source, it has
        // "removeObtainedGamePieces(arena)".
        // It stores pieces in "gamePiecesToRemove".
        // It doesn't seem to have a public method to "pop" one piece for the shooter.
        // We might need to modify IntakeSimulation3D or just skip this logic for now if
        // not strictly needed,
        // OR assuming IntakeSimulation3D handles "removal" by destroying them.
        // If we want transfer, we should add a method to IntakeSimulation3D.

        // For now, let's assume valid intake source integration requires updating
        // IntakeSimulation3D.
        // Since I'm creating this file, I can try to use what's available.
        // IntakeSimulation3D has `removeObtainedGamePieces`.
        // I will just rely on manual transfer or update `IntakeSimulation3D` if I can.

        if (intakeSource != null && gamePiecesLoaded < capacity) {
            if (intakeSource.obtainGamePieceFromIntake()) {
                gamePiecesLoaded++;
            }
        }
    }
}
