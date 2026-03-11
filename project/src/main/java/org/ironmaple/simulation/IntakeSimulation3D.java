package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Queue;
import java.util.function.Predicate;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation3D;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation3D;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Simulates an Intake Mechanism in 3D.</h1>
 *
 * <p>This class simulates a 3D intake attached to a 3D drivetrain. It detects game pieces within its volume and
 * "collects" them if enabled.
 */
public class IntakeSimulation3D implements SimulatedArena3D.Simulatable {
    private final String targetedGamePieceType;
    private final AbstractDriveTrainSimulation3D drivetrain;
    private final PhysicsShape intakeShape;
    private final Transform3d intakeOffset;
    private final int capacity;
    private int gamePiecesInIntakeCount;
    private boolean intakeRunning;
    private final Queue<GamePieceOnFieldSimulation3D> gamePiecesToRemove;
    private Predicate<GamePieceOnFieldSimulation3D> customIntakeCondition = gp -> true;

    private final PhysicsEngine physicsEngine;
    private final SimulatedArena3D arena;

    /**
     *
     *
     * <h2>Creates a 3D Intake Simulation.</h2>
     *
     * @param targetedGamePieceType the type of game piece to collect
     * @param drivetrain the drivetrain this intake is attached to
     * @param intakeShape the 3D shape of the intake volume
     * @param intakeOffset the offset of the intake center from the robot center
     * @param capacity the maximum number of game pieces
     * @param arena the simulation arena
     */
    public IntakeSimulation3D(
            String targetedGamePieceType,
            AbstractDriveTrainSimulation3D drivetrain,
            PhysicsShape intakeShape,
            Transform3d intakeOffset,
            int capacity,
            SimulatedArena3D arena) {
        this.targetedGamePieceType = targetedGamePieceType;
        this.drivetrain = drivetrain;
        this.intakeShape = intakeShape;
        this.intakeOffset = intakeOffset;
        this.capacity = capacity;
        this.gamePiecesInIntakeCount = 0;
        this.gamePiecesToRemove = new ArrayDeque<>();
        this.physicsEngine = arena.getPhysicsEngine();
        this.arena = arena;

        arena.addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Turns the Intake On.</h2>
     */
    public void startIntake() {
        this.intakeRunning = true;
    }

    /**
     *
     *
     * <h2>Turns the Intake Off.</h2>
     */
    public void stopIntake() {
        this.intakeRunning = false;
    }

    /**
     * Gets the number of game pieces currently in the intake.
     *
     * @return number of game pieces
     */
    public int getGamePiecesInIntake() {
        return gamePiecesInIntakeCount;
    }

    /**
     * Checks if the intake is running.
     *
     * @return true if running
     */
    public boolean isRunning() {
        return intakeRunning;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (!intakeRunning || gamePiecesInIntakeCount >= capacity) return;

        // Calculate current intake pose
        Pose3d robotPose = drivetrain.getSimulatedDriveTrainPose3d();
        Pose3d intakePose = robotPose.plus(intakeOffset);

        // Check for overlaps
        List<PhysicsBody> overlaps = physicsEngine.getOverlappingBodies(intakeShape, intakePose);

        // Debug logging (once per second, ~every 50 ticks at 50Hz)
        // if (subTickNum == 0) {
        // System.out.printf(
        // "[IntakeSimulation3D] Running=%b, IntakePose=(%.2f, %.2f, %.2f), Overlaps=%d,
        // InIntake=%d%n",
        // intakeRunning,
        // intakePose.getX(),
        // intakePose.getY(),
        // intakePose.getZ(),
        // overlaps.size(),
        // gamePiecesInIntakeCount);
        // }

        for (PhysicsBody body : overlaps) {
            Object userData = body.getUserData();
            if (userData instanceof GamePieceOnFieldSimulation3D gp) {
                if (gp.getType().equals(targetedGamePieceType) && customIntakeCondition.test(gp)) {
                    if (!gamePiecesToRemove.contains(gp)) {
                        gamePiecesToRemove.add(gp);
                        gamePiecesInIntakeCount++;
                        System.out.printf(
                                "[IntakeSimulation3D] Collected game piece! Type=%s, Now have %d%n",
                                gp.getType(), gamePiecesInIntakeCount);
                        if (gamePiecesInIntakeCount >= capacity) break;
                    }
                }
            } else {
                // Log what types of bodies we're finding (to debug if game pieces aren't being
                // recognized)
                if (subTickNum == 0) {
                    System.out.printf(
                            "[IntakeSimulation3D] Overlap detected but userData=%s (not a game piece)%n",
                            userData != null ? userData.getClass().getSimpleName() : "null");
                }
            }
        }

        // Immediately remove collected pieces from the field
        removeObtainedGamePiecesInternal();
    }

    /** Internal method to remove pieces from physics engine AND display list. */
    private void removeObtainedGamePiecesInternal() {
        while (!gamePiecesToRemove.isEmpty()) {
            GamePieceOnFieldSimulation3D gp = gamePiecesToRemove.poll();
            // Use arena.removePiece to properly clean up both physics and display
            arena.removePiece(gp);
            System.out.printf("[IntakeSimulation3D] Removed game piece from field (physics + display)%n");
        }
    }

    /**
     *
     *
     * <h2>Removes obtained game pieces.</h2>
     *
     * <p>Called by arena or manually.
     */
    /**
     *
     *
     * <h2>Removes obtained game pieces.</h2>
     *
     * <p>Called by arena or manually.
     */
    public void removeObtainedGamePieces(SimulatedArena3D arena) {
        while (!gamePiecesToRemove.isEmpty()) {
            GamePieceOnFieldSimulation3D gp = gamePiecesToRemove.poll();
            arena.getPhysicsEngine().removeBody(gp.getPhysicsBody());
            // Add logic to move to internal storage or notify manager
        }
    }

    /**
     *
     *
     * <h2>Obtains a game piece from the intake.</h2>
     *
     * <p>Called by consumers (like ShooterSimulation3D) to take a piece from the intake.
     *
     * @return true if a piece was successfully obtained
     */
    public boolean obtainGamePieceFromIntake() {
        if (gamePiecesInIntakeCount > 0) {
            gamePiecesInIntakeCount--;
            // If we have specific pieces tracked, we might want to remove them from the
            // queue
            // But since 'gamePiecesToRemove' tracks pieces to be removed from FIELD,
            // we should probably ensure they ARE removed from field if they are transferred
            // to shooter.
            // If 'obtain' is called, it implies the piece is entering the robot system.
            // So we should effectively ensure the piece is gone from the field.

            // If gamePiecesToRemove is used solely for "cleanup from field",
            // and we rely on 'removeObtainedGamePieces' being called periodically (e.g. by
            // subsystem or arena),
            // then we just decrement the count here representing "in-robot storage"
            // transfer.

            // However, to be safe and consistent:
            // If the intake logic is "accumulate in intake until full",
            // and shooter says "give me one", intake gives one.
            return true;
        }
        return false;
    }
}
