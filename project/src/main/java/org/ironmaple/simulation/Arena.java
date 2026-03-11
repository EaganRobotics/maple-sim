package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h2>Interface for Simulation Arenas.</h2>
 *
 * <p>Common interface for both 2D ({@link SimulatedArena}) and 3D ({@link SimulatedArena3D}) arenas. Allows game pieces
 * (like projectiles) to interact with the arena without knowing the specific implementation.
 */
public interface Arena {
    /** Represents a custom simulation to be updated during each simulation sub-tick. */
    interface Simulatable {
        void simulationSubTick(int subTickNum);
    }

    /**
     * Registers a custom simulation.
     *
     * @param simulatable the custom simulation to register
     */
    void addCustomSimulation(Simulatable simulatable);

    /**
     * Spawns a game piece on the field.
     *
     * @param info the game piece configuration
     * @param pose the initial pose
     * @param velocity the initial velocity
     */
    void spawnGamePieceOnField(GamePieceOnFieldSimulation.GamePieceInfo info, Pose3d pose, Translation3d velocity);

    /**
     * Adds a game piece projectile to the simulation.
     *
     * @param projectile the projectile to add
     */
    void addGamePieceProjectile(org.ironmaple.simulation.gamepieces.GamePieceProjectile projectile);

    /**
     * Removes a game piece from the simulation.
     *
     * @param piece the piece to remove
     * @return true if removed
     */
    boolean removePiece(GamePiece piece);

    /**
     * Gets game pieces by type.
     *
     * @param type the type of game piece
     * @return list of game pieces
     */
    List<GamePiece> getGamePiecesByType(String type);

    /**
     * Gets game pieces by type as an array.
     *
     * @param type the type of game piece
     * @return array of game pieces
     */
    GamePiece[] getGamePiecesByTypeAsArray(String type);

    /**
     * Adds to the score of the specified team.
     *
     * @param isBlue whether to add to the blue team
     * @param points points to add
     */
    void addToScore(boolean isBlue, int points);

    /**
     * Adds value to match breakdown.
     *
     * @param isBlue whether to add to the blue team
     * @param key the key
     * @param value the value
     */
    void addValueToMatchBreakdown(boolean isBlue, String key, double value);

    void addValueToMatchBreakdown(boolean isBlue, String key, int value);

    /**
     * Replaces value in match breakdown.
     *
     * @param isBlue whether to add to the blue team
     * @param key the key
     * @param value the value
     */
    void replaceValueInMatchBreakDown(boolean isBlue, String key, double value);

    void replaceValueInMatchBreakDown(boolean isBlue, String key, int value);

    /**
     * Registers that a game piece has been scored.
     *
     * @param piece the game piece
     * @param goal the goal object (opaque)
     */
    void registerGamePieceScored(GamePiece piece, Object goal);

    /**
     * Registers a source for game piece visualization (e.g. pieces inside a goal).
     *
     * @param type the game piece type
     * @param source a consumer that accepts a list of poses to add to
     */
    void registerGamePieceVisualizationSource(String type, java.util.function.Consumer<List<Pose3d>> source);

    /**
     * Gets 3D poses of all game pieces of a specific type.
     *
     * @param type the game piece type
     * @return list of poses
     */
    List<Pose3d> getGamePiecesPosesByType(String type);
}
