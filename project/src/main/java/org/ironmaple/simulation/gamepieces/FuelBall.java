package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.physics.PhysicsShape;
import org.ironmaple.simulation.physics.jolt.JoltPhysicsEngine;

/**
 *
 *
 * <h1>Fuel Ball Game Piece (2026)</h1>
 *
 * <p>Represents the high-density foam ball "FUEL" for the 2026 game.
 *
 * <p>Physical Properties:
 *
 * <ul>
 *   <li>Diameter: ~5.91 in (0.15m) -> Radius 0.075m
 *   <li>Mass: 0.23 kg (~0.5 lbs)
 *   <li>Material: High-Density Foam (Low Restitution, High Friction)
 * </ul>
 */
public class FuelBall extends GamePieceOnFieldSimulation3D {
    // Physical Parameters
    private static final String TYPE = "Fuel";
    private static final double BASE_RADIUS_METERS = 0.075;
    private static final Mass MASS = Units.Kilograms.of(0.23);
    private static final double LINEAR_DAMPING = 1.0;
    private static final double ANGULAR_DAMPING = 4.0;
    private static final double RESTITUTION = 0.15;
    private static final double FRICTION = 0.9;

    /** Creation info factory. Note: We create a fresh shape per ball to support radius variance. */
    private static GamePieceInfo3D createInfo(SimulatedArena3D arena) {
        // Randomize radius by +/- 2%
        double variance = 1.0 + (Math.random() * 0.04 - 0.02);
        double radius = BASE_RADIUS_METERS * variance;

        PhysicsShape shape = arena.getPhysicsEngine().createSphereShape(radius);

        return new GamePieceInfo3D(TYPE, shape, MASS, FRICTION, LINEAR_DAMPING, ANGULAR_DAMPING, RESTITUTION);
    }

    /**
     * Creates a Fuel Ball simulation.
     *
     * @param arena the simulation arena
     * @param initialPose the initial pose
     */
    public FuelBall(SimulatedArena3D arena, Pose3d initialPose) {
        super(arena, createInfo(arena), initialPose);

        // Apply friction (Note: GamePieceInfo3D doesn't currently carry friction, so we
        // set it on body)
        // Check if physics body is valid
        if (getPhysicsBody() != null) {
            // Friction, Restitution, and Damping are now applied via GamePieceInfo3D at
            // creation time.

            // Defualt to Tier 2 (Performance)
            setPerformanceTier(2);
        }
    }

    /**
     * Sets the performance tier for this game piece.
     *
     * <ul>
     *   <li><b>Tier 1 (High Fidelity):</b> Enables contact reporting. Used for active balls (in intake, near robot).
     *   <li><b>Tier 2 (Bulk/Background):</b> Disables contact reporting. Used for the bulk of field elements.
     * </ul>
     *
     * @param tier the tier (1 or 2)
     */
    public void setPerformanceTier(int tier) {
        if (getPhysicsBody() == null) return;

        switch (tier) {
            case 1:
                getPhysicsBody().setContactReporting(true);
                // Ensure it collides with everything
                // Using MOVING layer for now, but could switch to a high-priority layer if
                // implemented
                getPhysicsBody().setCollisionLayer(JoltPhysicsEngine.OBJ_LAYER_MOVING);
                break;
            case 2:
            default:
                getPhysicsBody().setContactReporting(false);
                // Keep largely standard collision
                getPhysicsBody().setCollisionLayer(JoltPhysicsEngine.OBJ_LAYER_MOVING);
                break;
        }
    }
}
