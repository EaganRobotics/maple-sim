package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.ironmaple.utils.FieldMirroringUtils.flip;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.gamepieces.FuelBall;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation3D;
import org.ironmaple.simulation.physics.PhysicsShape;

public class Arena2026Rebuilt3D extends SimulatedArena3D implements Arena2026 {
    protected final boolean useAdvancedFuelPhysics;
    protected boolean shouldClock = true;
    protected double clock = 0;
    protected boolean blueIsOnClock = Math.random() < 0.5;

    protected DoublePublisher phaseClockPublisher =
            genericInfoTable.getDoubleTopic("Time left in current phase").publish();

    protected BooleanPublisher redActivePublisher =
            redTable.getBooleanTopic("Red is active").publish();
    protected BooleanPublisher blueActivePublisher =
            blueTable.getBooleanTopic("Blue is active").publish();

    protected RebuiltHub blueHub;
    protected RebuiltHub redHub;

    protected RebuiltOutpost blueOutpost;
    protected RebuiltOutpost redOutpost;

    // Field Dimensions
    private static final double FIELD_WIDTH = 17.548;
    private static final double FIELD_HEIGHT = 8.052;
    private Translation2d fuelZoneCenter = new Translation2d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2);
    private int neutralFuelCount = 408;
    private double fuelSeparationGap = 0.001; // 1mm

    public Arena2026Rebuilt3D() {
        this(false);
    }

    public Arena2026Rebuilt3D(boolean useAdvancedFuelPhysics) {
        super(new RebuiltFieldObstacleMap3D());
        this.useAdvancedFuelPhysics = useAdvancedFuelPhysics;

        blueHub = new RebuiltHub(this, true);
        this.addCustomSimulation(blueHub);

        redHub = new RebuiltHub(this, false);
        this.addCustomSimulation(redHub);

        blueOutpost = new RebuiltOutpost(this, true);
        this.addCustomSimulation(blueOutpost);

        redOutpost = new RebuiltOutpost(this, false);
        this.addCustomSimulation(redOutpost);
    }

    public void setNeutralFuelCount(int neutralFuelCount) {
        this.neutralFuelCount = Math.min(neutralFuelCount, 600);
    }

    /**
     *
     *
     * <h2>Adds a game piece too the arena with a certain random variance.</h2>
     *
     * This method is useful for certain spawners like the return cutes on the hub to prevent the game pieces from being
     * returned to the exact same position every time.
     *
     * @param piecePose the position of the piece to spawn
     * @param yaw the yaw
     * @param height the height
     * @param speed the speed
     * @param pitch the pitch
     * @param xVariance The max amount of variance that should be added too the x coordinate of the game piece.
     * @param yVariance The max amount of variance that should be added too the y coordinate of the game piece.
     * @param yawVariance The max amount of variance that should be added too the yaw of the game piece.
     * @param speedVariance The max amount of variance that should be added too the speed of the game piece.
     * @param pitchVariance The max amount of variance that should be added too the pitch of the game piece.
     */
    public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double xVariance,
            double yVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        var projectile = new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
                new Translation2d(),
                new ChassisSpeeds(),
                yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance)));
        this.addGamePieceProjectile(projectile);
        // projectile.launch(); // addGamePieceProjectile calls launch
    }

    @Override
    public void placeGamePiecesOnField() {
        // Fuel diameter - 2 * 3.5 inches = 7 inches = ~0.18m, using 0.15 for simulation
        // Fuel diameter - 2 * 3.5 inches = 7 inches = ~0.18m, using 0.15 for simulation
        // Account for 2% variance in radius (4% in diameter) to prevent layout overlap
        double nominalDiameter = 0.15;
        double maxDiameter = nominalDiameter * 1.04;

        double fuelRadius = nominalDiameter / 2;
        // Spacing uses MAX possible diameter to ensure no overlap + gap
        double spacing = maxDiameter + fuelSeparationGap;

        double boundingBoxWidth = edu.wpi.first.units.Units.Inches.of(206).in(edu.wpi.first.units.Units.Meters);
        double boundingBoxDepth = edu.wpi.first.units.Units.Inches.of(72).in(edu.wpi.first.units.Units.Meters);

        java.util.List<Translation2d> q1 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q2 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q3 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q4 = new java.util.ArrayList<>();

        // Generate points for one quadrant (Q1) starting from the center outward
        double dividerWidth = edu.wpi.first.units.Units.Inches.of(2).in(edu.wpi.first.units.Units.Meters);
        double startX = dividerWidth / 2 + fuelSeparationGap + fuelRadius;
        double startY = dividerWidth / 2 + fuelSeparationGap + fuelRadius;

        for (double x = startX; x + fuelRadius <= boundingBoxDepth / 2; x += spacing) {
            for (double y = startY; y + fuelRadius <= boundingBoxWidth / 2; y += spacing) {
                // Add mirrored positions to all quadrants
                q1.add(fuelZoneCenter.plus(new Translation2d(x, y))); // Q1
                q2.add(fuelZoneCenter.plus(new Translation2d(-x, y))); // Q2
                q3.add(fuelZoneCenter.plus(new Translation2d(-x, -y))); // Q3
                q4.add(fuelZoneCenter.plus(new Translation2d(x, -y))); // Q4
            }
        }

        // Round robin placement
        java.util.List<java.util.List<Translation2d>> quadrants = java.util.List.of(q1, q2, q3, q4);
        int added = 0;
        boolean piecesAvailable = true;

        java.util.Comparator<Translation2d> distComparator =
                java.util.Comparator.comparingDouble(p -> p.getDistance(fuelZoneCenter));
        for (java.util.List<Translation2d> q : quadrants) {
            q.sort(distComparator);
        }

        int[] indices = new int[4];

        // Define 3D fuel shape - Sphere
        PhysicsShape fuelShape = physicsEngine.createSphereShape(fuelRadius);
        GamePieceOnFieldSimulation3D.GamePieceInfo3D fuelInfo = new GamePieceOnFieldSimulation3D.GamePieceInfo3D(
                "Fuel",
                fuelShape,
                Kilograms.of(0.2), // map mass
                0.8, // friction
                1.0, // linear damping
                4.0, // angular damping (high rolling resistance)
                0.1 // restitution
                );

        while (added < neutralFuelCount && piecesAvailable) {
            piecesAvailable = false;
            for (int i = 0; i < 4; i++) {
                if (added >= neutralFuelCount) break;

                java.util.List<Translation2d> q = quadrants.get(i);
                if (indices[i] < q.size()) {
                    Translation2d pos2d = q.get(indices[i]);
                    GamePieceOnFieldSimulation3D piece;
                    if (useAdvancedFuelPhysics) {
                        piece = new FuelBall(
                                this,
                                new Pose3d(
                                        pos2d.getX(),
                                        pos2d.getY(),
                                        fuelRadius * 1.03,
                                        new edu.wpi.first.math.geometry.Rotation3d()));
                    } else {
                        // Spawn 3D piece (Standard)
                        piece = new GamePieceOnFieldSimulation3D(
                                this,
                                fuelInfo,
                                new Pose3d(
                                        pos2d.getX(),
                                        pos2d.getY(),
                                        fuelRadius * 1.03,
                                        new edu.wpi.first.math.geometry.Rotation3d()));
                    }
                    // Register dynamic body
                    dynamicBodies.put(piece, piece.getPhysicsBody());

                    indices[i]++;
                    added++;
                    piecesAvailable = true;
                }
            }
        }

        // Depot Placement
        Translation2d depotOrigin = new Translation2d(0.071, 5.584);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++) {
                double x = depotOrigin.getX() + (i * spacing);
                double y = depotOrigin.getY() + (j * spacing);

                // Blue side
                Translation2d bluePos = new Translation2d(x, y);
                GamePieceOnFieldSimulation3D pieceBlue;
                if (useAdvancedFuelPhysics) {
                    pieceBlue = new FuelBall(
                            this,
                            new Pose3d(
                                    bluePos.getX(),
                                    bluePos.getY(),
                                    // Spawn Z: Nominal radius + 3% (2% variance + 1% safety) to prevent floor
                                    // penetration
                                    fuelRadius * 1.03,
                                    new edu.wpi.first.math.geometry.Rotation3d()));
                } else {
                    pieceBlue = new GamePieceOnFieldSimulation3D(
                            this,
                            fuelInfo,
                            new Pose3d(
                                    bluePos.getX(),
                                    bluePos.getY(),
                                    // Spawn Z: Nominal radius + 3% (2% variance + 1% safety) to prevent floor
                                    // penetration
                                    fuelRadius * 1.03,
                                    new edu.wpi.first.math.geometry.Rotation3d()));
                }
                dynamicBodies.put(pieceBlue, pieceBlue.getPhysicsBody());

                // Red side (mirrored)
                Translation2d redPos = flip(bluePos);
                GamePieceOnFieldSimulation3D pieceRed;
                if (useAdvancedFuelPhysics) {
                    pieceRed = new FuelBall(
                            this,
                            new Pose3d(
                                    redPos.getX(),
                                    redPos.getY(),
                                    // Spawn Z: Nominal radius + 3% (2% variance + 1% safety) to prevent floor
                                    // penetration
                                    fuelRadius * 1.03,
                                    new edu.wpi.first.math.geometry.Rotation3d()));
                } else {
                    pieceRed = new GamePieceOnFieldSimulation3D(
                            this,
                            fuelInfo,
                            new Pose3d(
                                    redPos.getX(),
                                    redPos.getY(),
                                    // Spawn Z: Nominal radius + 3% (2% variance + 1% safety) to prevent floor
                                    // penetration
                                    fuelRadius * 1.03,
                                    new edu.wpi.first.math.geometry.Rotation3d()));
                }
                dynamicBodies.put(pieceRed, pieceRed.getPhysicsBody());
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }

    @Override
    protected void simulationSubTick(int tickNum) {
        if (shouldClock && !DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            clock -= getSimulationDt().in(Units.Seconds);

            if (clock <= 0) {
                clock = 25;
                blueIsOnClock = !blueIsOnClock;
            }
        } else {
            clock = 25;
            String fmsMessage = DriverStation.getGameSpecificMessage();
            if (fmsMessage != null) {
                if (fmsMessage.contains("B")) blueIsOnClock = false;
                else if (fmsMessage.contains("R")) blueIsOnClock = true;
            }
        }

        phaseClockPublisher.set((clock));

        super.simulationSubTick(tickNum);

        blueActivePublisher.set(isActive(true));
        redActivePublisher.set(isActive(false));
    }

    public boolean isActive(boolean isBlue) {
        if (isBlue) {
            return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        } else {
            return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        }
    }

    public void setShouldRunClock(boolean shouldRunClock) {
        shouldClock = shouldRunClock;
    }

    public void outpostDump(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).dump();
    }

    public void outpostThrowForGoal(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).throwForGoal();
    }

    public void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
        (isBlue ? blueOutpost : redOutpost).throwFuel(throwYaw, throwPitch, speed);
    }

    public static double randomInRange(double variance) {
        return (Math.random() - 0.5) * variance;
    }
}
