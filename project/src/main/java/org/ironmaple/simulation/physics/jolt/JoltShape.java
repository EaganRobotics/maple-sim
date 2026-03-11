package org.ironmaple.simulation.physics.jolt;

import com.github.stephengold.joltjni.readonly.ConstShape;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Jolt Physics Shape Implementation</h1>
 *
 * <p>Implements the {@link PhysicsShape} interface by wrapping a Jolt {@link ConstShape}.
 */
public class JoltShape implements PhysicsShape {
    private final ConstShape shape;
    private final ShapeType type;

    /**
     *
     *
     * <h2>Creates a JoltShape Wrapper.</h2>
     *
     * @param shape the underlying Jolt shape
     * @param type the shape type
     */
    public JoltShape(ConstShape shape, ShapeType type) {
        this.shape = shape;
        this.type = type;
    }

    /**
     *
     *
     * <h2>Gets the Underlying Jolt Shape.</h2>
     *
     * @return the Jolt shape
     */
    public ConstShape getShape() {
        return shape;
    }

    @Override
    public ShapeType getType() {
        return type;
    }
}
