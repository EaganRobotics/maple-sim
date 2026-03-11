package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import org.ironmaple.simulation.debugging.SimDebugLogger;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation3D;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>3D Simulation World</h1>
 *
 * <p>A 3D physics simulation arena using Jolt Physics. This provides high-fidelity simulation with:
 *
 * <ul>
 *   <li>Full 3D rigid body dynamics
 *   <li>Realistic collision detection with complex meshes
 *   <li>Raycasting for suspension and ground detection
 *   <li>Support for slopes, ramps, and inclined surfaces
 * </ul>
 *
 * <p>This class provides a similar API to {@link SimulatedArena} but uses 3D physics internally.
 */
public abstract class SimulatedArena3D implements Arena {
    /** Whether to allow the simulation to run a real robot. HIGHLY RECOMMENDED to be turned OFF */
    @Deprecated(forRemoval = true)
    public static boolean ALLOW_CREATION_ON_REAL_ROBOT = false;

    // ==================== Singleton Management ====================
    private static SimulatedArena3D instance;
    private static int defaultJoltWorkerThreads = 4;

    /**
     * Gets the arena singleton instance.
     *
     * @return the arena instance, or null if not yet created
     */
    public static SimulatedArena3D getInstance() {
        return instance;
    }

    /**
     * Configures the Jolt Physics Engine worker thread count.
     *
     * <p>Must be called BEFORE the arena is created. Has no effect after creation.
     *
     * @param workerThreads Number of worker threads (default 4)
     */
    public static void configureJolt(int workerThreads) {
        if (instance != null) {
            throw new IllegalStateException(
                    "Cannot configure physics after arena is created! Call configureJolt() before creating the arena.");
        }
        defaultJoltWorkerThreads = workerThreads;
    }

    /**
     * Resets the singleton instance (for testing purposes).
     *
     * <p>Shuts down the existing arena and allows a new one to be created.
     */
    public static synchronized void resetInstance() {
        if (instance != null) {
            instance.shutdown();
            instance = null;
        }
        defaultJoltWorkerThreads = 4;
    }

    // ==================== End Singleton Management ====================

    protected int redScore = 0;
    protected int blueScore = 0;
    protected double matchClock = 0;

    public Map<String, Double> redScoringBreakdown = new Hashtable<String, Double>();
    public Map<String, Double> blueScoringBreakdown = new Hashtable<String, Double>();
    protected Map<String, DoublePublisher> redPublishers = new Hashtable<String, DoublePublisher>();
    protected Map<String, DoublePublisher> bluePublishers = new Hashtable<String, DoublePublisher>();

    public NetworkTable redTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim3D/MatchData/Breakdown/Red Alliance");
    public NetworkTable blueTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim3D/MatchData/Breakdown/Blue Alliance");
    public NetworkTable genericInfoTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim3D/MatchData/Breakdown");

    public DoublePublisher matchClockPublisher =
            genericInfoTable.getDoubleTopic("Match Clock").publish();

    public static BooleanPublisher resetFieldPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim3D/MatchData")
            .getBooleanTopic("Reset Field")
            .publish();

    public static BooleanSubscriber resetFieldSubscriber =
            resetFieldPublisher.getTopic().subscribe(false);

    Boolean shouldPublishMatchBreakdown = true;

    /** The number of sub-ticks the simulator will run in each robot period. */
    private static int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;

    public static int getSimulationSubTicksIn1Period() {
        return SIMULATION_SUB_TICKS_IN_1_PERIOD;
    }

    /** The period length of each sub-tick, in seconds. */
    private static Time SIMULATION_DT =
            edu.wpi.first.units.Units.Seconds.of(TimedRobot.kDefaultPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD);

    public static Time getSimulationDt() {
        return SIMULATION_DT;
    }

    /** Last measured physics CPU time in seconds. */
    private double lastPhysicsCpuTimeSeconds = 0.0;

    /** The 3D physics backend. */
    protected final org.ironmaple.simulation.physics.PhysicsBackend physicsBackend;

    /** The underlying physics engine for direct access. */
    protected final PhysicsEngine physicsEngine;

    /** Registered dynamic bodies (robots, game pieces). */
    protected final Map<Object, PhysicsBody> dynamicBodies = new HashMap<>();

    /** Registered static bodies (field elements). */
    protected final List<PhysicsBody> staticBodies = new ArrayList<>();

    /** Registered projectiles. */
    protected final Set<GamePieceProjectile> projectiles = new HashSet<>();

    /** Custom simulations. */
    protected final List<Simulatable> customSimulations = new ArrayList<>();

    // Use Arena.Simulatable instead of defining it here.
    // public interface Simulatable ... (removed)

    /**
     *
     *
     * <h2>Constructs a new 3D simulation arena.</h2>
     *
     * @param fieldMap the field map defining static obstacles
     */
    protected SimulatedArena3D(FieldMap3D fieldMap) {
        // Fail-fast: prevent multiple arenas
        if (instance != null) {
            throw new IllegalStateException("SimulatedArena3D already created! Only one arena instance is allowed. "
                    + "Use SimulatedArena3D.getInstance() to access the existing arena, "
                    + "or call SimulatedArena3D.resetInstance() first if you need a new arena.");
        }

        if (RobotBase.isReal()) {
            throw new IllegalStateException("MapleSim3D cannot be instantiated on a real robot! "
                    + "Ensure you only initialize the arena in simulation. Example:\n"
                    + "if (RobotBase.isSimulation()) {\n"
                    + "    arena = new MySimulatedArena();\n"
                    + "}");
        }

        // Register as singleton immediately
        instance = this;

        org.ironmaple.simulation.physics.jolt.JoltBackend joltBackend =
                new org.ironmaple.simulation.physics.jolt.JoltBackend();
        joltBackend.setWorkerThreadCount(defaultJoltWorkerThreads);
        this.physicsBackend = joltBackend;

        this.physicsBackend.initialize();
        this.physicsEngine = joltBackend.getEngine();

        // Set gravity (downward)
        physicsBackend.setGravity(new Translation3d(0, 0, -9.81));

        // Add field obstacles
        for (FieldMap3D.Obstacle obstacle : fieldMap.getObstacles()) {
            PhysicsShape shape = obstacle.shape();
            // If shape is null, check for mesh resource or halfExtents
            if (shape == null && obstacle.meshResourcePath() != null) {
                try {
                    shape = physicsEngine.createCompoundShapeFromMesh(obstacle.meshResourcePath());
                } catch (Exception e) {
                    DriverStation.reportError(
                            "Failed to load mesh obstacle: " + obstacle.meshResourcePath(), e.getStackTrace());
                }
            } else if (shape == null && obstacle.halfExtents() != null) {
                shape = physicsEngine.createBoxShape(obstacle.halfExtents());
            }

            if (shape != null) {
                PhysicsBody body = physicsEngine.createStaticBody(shape, obstacle.pose());
                staticBodies.add(body);
                System.out.println("[MapleSim3D] Created static body: "
                        + (obstacle.meshResourcePath() != null ? obstacle.meshResourcePath() : "Primitives"));
                System.out.println("[MapleSim3D] Global Pose: " + obstacle.pose());
            } else {
                System.err.println("[MapleSim3D] Failed to create shape for obstacle: "
                        + (obstacle.meshResourcePath() != null ? obstacle.meshResourcePath() : "Primitives"));
            }
        }

        setupValueForMatchBreakdown("TotalScore");
        setupValueForMatchBreakdown("TeleopScore");
        setupValueForMatchBreakdown("Auto/AutoScore");
        resetFieldPublisher.set(false);
        if (instance == null) instance = this;
    }

    /**
     *
     *
     * <h2>Overrides the Timing Configurations.</h2>
     *
     * @param robotPeriod the time between two calls of {@link #simulationPeriodic()}
     * @param simulationSubTicksPerPeriod the number of sub-ticks per period
     */
    public static synchronized void overrideSimulationTimings(Time robotPeriod, int simulationSubTicksPerPeriod) {
        SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
        SIMULATION_DT = robotPeriod.div(SIMULATION_SUB_TICKS_IN_1_PERIOD);
    }

    /**
     *
     *
     * <h2>Registers a custom simulation.</h2>
     *
     * @param simulatable the custom simulation to register
     */
    public synchronized void addCustomSimulation(Simulatable simulatable) {
        this.customSimulations.add(simulatable);
    }

    /**
     *
     *
     * <h2>Updates the simulation world.</h2>
     *
     * <p>Call this ONCE in TimedRobot.simulationPeriodic().
     *
     * <p>Runs multiple sub-ticks per period.
     */
    public synchronized void simulationPeriodic() {
        synchronized (SimulatedArena3D.class) {
            final long t0 = System.nanoTime();

            // Synchronous mode: Original behavior with sub-ticks
            for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++) {
                simulationSubTick(i);
            }
            matchClock += getSimulationDt().in(Units.Seconds) * SIMULATION_SUB_TICKS_IN_1_PERIOD;

            double cpuTimeMs = (System.nanoTime() - t0) / 1_000_000.0;
            this.lastPhysicsCpuTimeSeconds = cpuTimeMs / 1000.0;

            SmartDashboard.putNumber("MapleSim3D/PhysicsEngineCPUTimeMS", cpuTimeMs);
            SimDebugLogger.logPerformance(String.format("Sim Periodic Time: %.3f ms", cpuTimeMs));

            if (resetFieldSubscriber.get()) {
                resetFieldForAuto();
                resetFieldPublisher.set(false);
                matchClock = 0;
            }
        }
    }

    /**
     *
     *
     * <h2>Processes a single simulation sub-tick.</h2>
     */
    protected void simulationSubTick(int subTickNum) {
        SimulatedBattery.simulationSubTick();

        // Update projectiles
        GamePieceProjectile.updateGamePieceProjectiles(this, projectiles);

        // Step the physics engine
        physicsBackend.step(SIMULATION_DT);

        // Run custom simulations
        for (Simulatable customSimulation : customSimulations) {
            customSimulation.simulationSubTick(subTickNum);
        }

        replaceValueInMatchBreakDown(true, "TotalScore", blueScore);
        replaceValueInMatchBreakDown(false, "TotalScore", redScore);

        if (shouldPublishMatchBreakdown) {
            publishBreakdown();
            matchClockPublisher.set(matchClock);
        }
    }

    /**
     *
     *
     * <h2>Performs a raycast.</h2>
     *
     * @param origin the ray origin
     * @param direction the ray direction
     * @param maxDistance the maximum distance to check
     * @return the raycast result, or empty if no hit
     */
    public Optional<PhysicsEngine.RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance) {
        return physicsEngine.raycast(origin, direction, maxDistance);
    }

    /**
     *
     *
     * <h2>Gets the underlying physics engine.</h2>
     *
     * @return the Bullet physics engine
     */
    public PhysicsEngine getPhysicsEngine() {
        return physicsEngine;
    }

    /**
     *
     *
     * <h2>Gets the last physics tick duration.</h2>
     *
     * <p>Returns the duration of the last simulationPeriodic call.
     *
     * @return duration in seconds
     */
    public double getLastPhysicsTickDuration() {
        return lastPhysicsCpuTimeSeconds;
    }

    /**
     *
     *
     * <h2>Shuts down the simulation.</h2>
     */
    public synchronized void shutdown() {
        physicsBackend.shutdown();
    }

    /**
     *
     *
     * <h2>Resets the field for autonomous mode.</h2>
     */
    public synchronized void resetFieldForAuto() {
        matchClock = 0;
        placeGamePiecesOnField();
    }

    /**
     *
     *
     * <h2>Places game pieces on the field.</h2>
     *
     * <p>Implemented by season-specific subclasses.
     */
    public abstract void placeGamePiecesOnField();

    @Override
    public void spawnGamePieceOnField(
            GamePieceOnFieldSimulation.GamePieceInfo info, Pose3d pose, Translation3d velocity) {
        // TODO: Implement proper shape mapping based on info.type()
        // For now, assume simple box/cylinder or use a default 3D info
        PhysicsShape shape = physicsEngine.createBoxShape(new Translation3d(0.1, 0.1, 0.1));

        GamePieceOnFieldSimulation3D.GamePieceInfo3D info3d = new GamePieceOnFieldSimulation3D.GamePieceInfo3D(
                info.type(), shape, info.gamePieceMass(), 0.8, 0.1, 0.1, 0.5);

        GamePieceOnFieldSimulation3D originalPiece = new GamePieceOnFieldSimulation3D(this, info3d, pose);
        originalPiece.getPhysicsBody().setLinearVelocityMPS(velocity);

        // Register body mapping?
        // dynamicBodies is Map<Object, PhysicsBody>.
        // We should add the piece as key?
        // But removePiece takes GamePiece.
        // GamePieceOnFieldSimulation3D IS a GamePiece.
        dynamicBodies.put(originalPiece, originalPiece.getPhysicsBody());
    }

    /**
     * Registers a game piece with the arena's dynamic bodies tracking. This ensures it can be removed properly later.
     *
     * @param piece the game piece to register
     */
    public void registerGamePiece(GamePieceOnFieldSimulation3D piece) {
        dynamicBodies.put(piece, piece.getPhysicsBody());
    }

    @Override
    public boolean removePiece(GamePiece piece) {
        if (projectiles.remove(piece)) {
            return true;
        }

        PhysicsBody body = dynamicBodies.remove(piece);
        if (body != null) {
            physicsEngine.removeBody(body);
            return true;
        }
        return false;
    }

    /**
     * Adds a projectile to the simulation.
     *
     * @param projectile the projectile to add
     */
    public void addGamePieceProjectile(GamePieceProjectile projectile) {
        this.projectiles.add(projectile);
        projectile.launch();
    }

    /** @return set of active projectiles */
    public Set<GamePieceProjectile> gamePieceLaunched() {
        return projectiles;
    }

    @Override
    public List<GamePiece> getGamePiecesByType(String type) {
        List<GamePiece> pieces = new ArrayList<>();
        // Check dynamic bodies
        for (Object key : dynamicBodies.keySet()) {
            if (key instanceof GamePieceOnFieldSimulation3D) {
                GamePieceOnFieldSimulation3D gp = (GamePieceOnFieldSimulation3D) key;
                if (gp.getType().equals(type)) {
                    pieces.add(gp);
                }
            }
        }
        // Check projectiles
        for (GamePieceProjectile projectile : projectiles) {
            if (projectile.gamePieceType.equals(type)) {
                pieces.add(projectile);
            }
        }
        return pieces;
    }

    @Override
    public void registerGamePieceScored(GamePiece piece, Object goal) {
        // For now, just remove the piece. Logic can be expanded.
        // We could also log it or create a visual effect.
        System.out.println("Game Piece Scored in 3D: " + piece.getType());
        // Do NOT remove here, the caller (Goal) should remove it or we let it be
        // handled there.
        // Goal calls removePiece().
        // We just need to handle side effects if any.
    }

    // ==================== Scoring Methods ====================

    public int getScore(boolean isBlue) {
        return isBlue ? blueScore : redScore;
    }

    public int getScore(Alliance allianceColor) {
        return getScore(allianceColor == Alliance.Blue);
    }

    public void addToScore(boolean isBlue, int toAdd) {
        if (isBlue) blueScore += toAdd;
        else redScore += toAdd;
        addValueToMatchBreakdown(isBlue, DriverStation.isAutonomous() ? "Auto/AutoScore" : "TeleopScore", toAdd);
    }

    public void enableBreakdownPublishing() {
        shouldPublishMatchBreakdown = true;
    }

    public void disableBreakdownPublishing() {
        shouldPublishMatchBreakdown = false;
    }

    protected void publishBreakdown() {
        for (String key : redScoringBreakdown.keySet()) {
            if (!redPublishers.containsKey(key))
                redPublishers.put(key, redTable.getDoubleTopic(key).publish());
            redPublishers.get(key).set(redScoringBreakdown.get(key));
        }
        for (String key : blueScoringBreakdown.keySet()) {
            if (!bluePublishers.containsKey(key))
                bluePublishers.put(key, blueTable.getDoubleTopic(key).publish());
            bluePublishers.get(key).set(blueScoringBreakdown.get(key));
        }
    }

    public void replaceValueInMatchBreakDown(boolean isBlueTeam, String valueKey, double value) {
        if (isBlueTeam) blueScoringBreakdown.put(valueKey, value);
        else redScoringBreakdown.put(valueKey, value);
    }

    public void replaceValueInMatchBreakDown(boolean isBlueTeam, String valueKey, Integer value) {
        replaceValueInMatchBreakDown(isBlueTeam, valueKey, (double) value);
    }

    public void replaceValueInMatchBreakDown(boolean isBlueTeam, String valueKey, int value) {
        replaceValueInMatchBreakDown(isBlueTeam, valueKey, (double) value);
    }

    public void registerGamePieceVisualizationSource(String type, java.util.function.Consumer<List<Pose3d>> source) {
        // In 3D engine, visualization is implicit in getGamePiecesPosesByType if
        // tracking works.
        // But if we have external sources (like stored inside robot/subsystems), we
        // need to track them.
        // We can use a simple list of sources.
        // However, SimulatedArena3D doesn't have GamePieceManager like 2D.
        // We need to implement it to support registering sources.
        if (!visualizationSources.containsKey(type)) {
            visualizationSources.put(type, new ArrayList<>());
        }
        visualizationSources.get(type).add(source);
    }

    protected final Map<String, List<java.util.function.Consumer<List<Pose3d>>>> visualizationSources = new HashMap<>();

    public List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = new ArrayList<>();
        // 1. Dynamic bodies
        for (Object key : dynamicBodies.keySet()) {
            if (key instanceof GamePieceOnFieldSimulation3D) {
                GamePieceOnFieldSimulation3D gp = (GamePieceOnFieldSimulation3D) key;
                if (gp.getType().equals(type)) {
                    poses.add(gp.getPose3d());
                }
            }
        }
        // 2. Projectiles
        for (GamePieceProjectile projectile : projectiles) {
            if (projectile.gamePieceType.equals(type)) {
                poses.add(projectile.getPose3d());
            }
        }
        // 3. Registered sources
        if (visualizationSources.containsKey(type)) {
            for (java.util.function.Consumer<List<Pose3d>> source : visualizationSources.get(type)) {
                source.accept(poses);
            }
        }
        return poses;
    }

    public void setupValueForMatchBreakdown(String valueKey) {
        replaceValueInMatchBreakDown(true, valueKey, 0);
        replaceValueInMatchBreakDown(false, valueKey, 0);
    }

    public void addValueToMatchBreakdown(boolean isBlueTeam, String valueKey, double toAdd) {
        if (isBlueTeam) {
            if (blueScoringBreakdown.get(valueKey) == null) blueScoringBreakdown.put(valueKey, toAdd);
            else blueScoringBreakdown.put(valueKey, blueScoringBreakdown.get(valueKey) + toAdd);
        } else {
            if (redScoringBreakdown.get(valueKey) == null) redScoringBreakdown.put(valueKey, toAdd);
            else redScoringBreakdown.put(valueKey, redScoringBreakdown.get(valueKey) + toAdd);
        }
    }

    /**
     *
     *
     * <h2>Obtains the 3D Poses of a Specific Type of Game Piece as an array.</h2>
     *
     * @see #getGamePiecesPosesByType(String)
     */
    public synchronized Pose3d[] getGamePiecePosesArrayByType(String type) {
        return getGamePiecesPosesByType(type).toArray(Pose3d[]::new);
    }

    /**
     *
     *
     * <h2>Obtains the 3D Poses of a Specific Type of Game Piece as an array.</h2>
     *
     * @see #getGamePiecesPosesByType(String)
     * @deprecated Use {@link #getGamePiecePosesArrayByType(String)} instead. This method name was confusing as it
     *     implied returning GamePiece objects.
     */
    @Deprecated
    public synchronized Pose3d[] getGamePiecesArrayByType(String type) {
        return getGamePiecePosesArrayByType(type);
    }

    /**
     *
     *
     * <h2>Returns all game pieces on the field of the specified type as an array</h2>
     *
     * @param type The string type to be selected.
     * @return The game pieces as an array of {@link GamePiece}
     */
    public synchronized GamePiece[] getGamePiecesByTypeAsArray(String type) {
        return getGamePiecesByType(type).toArray(GamePiece[]::new);
    }

    public void addValueToMatchBreakdown(boolean isBlueTeam, String valueKey, int toAdd) {
        addValueToMatchBreakdown(isBlueTeam, valueKey, (double) toAdd);
    }

    // ==================== 3D Field Map ====================

    /**
     *
     *
     * <h1>3D Field Map</h1>
     *
     * <p>Defines static obstacles for a 3D simulation field.
     */
    public static class FieldMap3D {
        private final List<Obstacle> obstacles = new ArrayList<>();

        /**
         *
         *
         * <h2>Represents a static obstacle.</h2>
         */
        public static class Obstacle {
            private final PhysicsShape shape;
            private final Pose3d pose;
            private final Translation3d halfExtents;
            private final String meshResourcePath;

            public Obstacle(PhysicsShape shape, Pose3d pose, Translation3d halfExtents) {
                this(shape, pose, halfExtents, null);
            }

            public Obstacle(PhysicsShape shape, Pose3d pose, Translation3d halfExtents, String meshResourcePath) {
                this.shape = shape;
                this.pose = pose;
                this.halfExtents = halfExtents;
                this.meshResourcePath = meshResourcePath;
            }

            public Obstacle(String meshResourcePath, Pose3d pose) {
                this(null, pose, null, meshResourcePath);
            }

            public PhysicsShape shape() {
                return shape;
            }

            public Pose3d pose() {
                return pose;
            }

            public Translation3d halfExtents() {
                return halfExtents;
            }

            public String meshResourcePath() {
                return meshResourcePath;
            }
        }

        /**
         *
         *
         * <h2>Adds a box obstacle.</h2>
         *
         * @param halfExtents the half-extents (half width, half depth, half height)
         * @param pose the pose of the obstacle
         */
        public void addBox(Translation3d halfExtents, Pose3d pose) {
            // Shape will be created when added to physics engine
            obstacles.add(new Obstacle(null, pose, halfExtents));
        }

        /**
         *
         *
         * <h2>Gets all obstacles.</h2>
         *
         * @return list of obstacles
         */
        public List<Obstacle> getObstacles() {
            return obstacles;
        }

        /**
         *
         *
         * <h2>Adds a rectangular wall obstacle.</h2>
         *
         * @param lengthX the length in X
         * @param widthY the width in Y
         * @param heightZ the height in Z
         * @param pose the center pose
         */
        protected void addRectangularObstacle(double lengthX, double widthY, double heightZ, Pose3d pose) {
            Translation3d halfExtents = new Translation3d(lengthX / 2, widthY / 2, heightZ / 2);
            obstacles.add(new Obstacle(null, pose, halfExtents));
        }

        /**
         *
         *
         * <h2>Adds a border line (wall segment).</h2>
         *
         * @param start the start point (2D, ground level)
         * @param end the end point (2D, ground level)
         * @param height the wall height
         * @param thickness the wall thickness
         */
        protected void addBorderLine(Translation2d start, Translation2d end, double height, double thickness) {
            double length = start.getDistance(end);
            Translation2d center = start.plus(end).div(2);
            double angle = Math.atan2(end.getY() - start.getY(), end.getX() - start.getX());

            Translation3d halfExtents = new Translation3d(length / 2, thickness / 2, height / 2);
            Pose3d pose = new Pose3d(
                    new Translation3d(center.getX(), center.getY(), height / 2),
                    new edu.wpi.first.math.geometry.Rotation3d(0, 0, angle));

            obstacles.add(new Obstacle(null, pose, halfExtents));
        }
    }
}
