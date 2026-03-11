package org.ironmaple.simulation.physics.bullet;

import com.jme3.bullet.collision.shapes.CollisionShape;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Bullet Physics Shape Implementation</h1>
 *
 * <p>Implements the {@link PhysicsShape} interface by wrapping a Bullet {@link CollisionShape}.
 */
public class BulletShape implements PhysicsShape {
    private final CollisionShape collisionShape;
    private final ShapeType type;

    /**
     *
     *
     * <h2>Creates a BulletShape Wrapper.</h2>
     *
     * @param collisionShape the underlying Bullet collision shape
     * @param type the shape type
     */
    public BulletShape(CollisionShape collisionShape, ShapeType type) {
        this.collisionShape = collisionShape;
        this.type = type;
    }

    /**
     *
     *
     * <h2>Gets the Underlying Bullet CollisionShape.</h2>
     *
     * @return the Bullet collision shape
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    @Override
    public ShapeType getType() {
        return type;
    }
}
