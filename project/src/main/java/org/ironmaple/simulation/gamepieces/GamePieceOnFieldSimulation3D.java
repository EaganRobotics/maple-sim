package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Simulates a Game Piece on the Field in 3D.</h1>
 *
 * <p>This class simulates a game piece using the 3D physics engine.
 */
public class GamePieceOnFieldSimulation3D implements GamePiece, SimulatedArena3D.Simulatable {
    private final String type;
    private final PhysicsBody physicsBody;
    private final Runnable callback;

    /**
     *
     *
     * <h2>Configuration Info for a 3D Game Piece.</h2>
     */
    public record GamePieceInfo3D(
            String type,
            PhysicsShape shape,
            Mass mass,
            double friction,
            double linearDamping,
            double angularDamping,
            double coefficientOfRestitution) {}

    /**
     *
     *
     * <h2>Creates a 3D Game Piece Simulation.</h2>
     *
     * @param arena the 3D arena
     * @param info the game piece configuration
     * @param initialPose the initial pose (3D)
     */
    public GamePieceOnFieldSimulation3D(SimulatedArena3D arena, GamePieceInfo3D info, Pose3d initialPose) {
        this.type = info.type();
        PhysicsEngine engine = arena.getPhysicsEngine();

        this.physicsBody = engine.createDynamicBody(
                info.shape(),
                info.mass().in(edu.wpi.first.units.Units.Kilograms),
                info.friction(),
                info.coefficientOfRestitution(),
                info.linearDamping(),
                info.angularDamping(),
                initialPose);
        this.physicsBody.setUserData(this);
        this.callback = () -> {}; // Default no-op callback

        // Register with arena to ensure it can be tracked and removed
        arena.registerGamePiece(this);

        // Register for sub-tick updates if needed (currently not needed for simple
        // rigid bodies, but good for future)
        arena.addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Creates a 3D Game Piece Simulation from 2D Pose.</h2>
     *
     * @param arena the 3D arena
     * @param info the game piece configuration
     * @param initialPose2d the initial 2D pose (assumes z is determined by physics/ground)
     */
    public GamePieceOnFieldSimulation3D(SimulatedArena3D arena, GamePieceInfo3D info, Pose2d initialPose2d) {
        this(
                arena,
                info,
                new Pose3d(
                        initialPose2d.getX(),
                        initialPose2d.getY(),
                        0.5, // Spawn slightly above ground to avoid clipping
                        new Rotation3d(0, 0, initialPose2d.getRotation().getRadians())));
    }

    @Override
    public Pose3d getPose3d() {
        if (physicsBody == null) return new Pose3d();
        return physicsBody.getPose3d();
    }

    @Override
    public String getType() {
        return type;
    }

    @Override
    public Translation3d getVelocity3dMPS() {
        if (physicsBody == null) return new Translation3d();
        return physicsBody.getLinearVelocityMPS();
    }

    @Override
    public void triggerHitTargeCallBack() {
        callback.run();
    }

    @Override
    public boolean isGrounded() {
        return true;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        // Can be used for custom logic like auto-removal if out of bounds
    }

    /**
     *
     *
     * <h2>Gets the underlying physics body.</h2>
     *
     * @return the physics body
     */
    public PhysicsBody getPhysicsBody() {
        return physicsBody;
    }
}
