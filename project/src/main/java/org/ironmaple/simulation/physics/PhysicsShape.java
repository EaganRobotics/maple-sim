package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h1>Physics Shape Abstraction</h1>
 *
 * <p>Represents a collision shape for 3D physics. This interface abstracts the underlying physics engine's shape
 * representation.
 *
 * <p>Common implementations include:
 *
 * <ul>
 *   <li>Box shapes
 *   <li>Sphere shapes
 *   <li>Cylinder shapes
 *   <li>Convex hull shapes (from mesh data)
 *   <li>Compound shapes (multiple primitives)
 * </ul>
 */
public interface PhysicsShape {

    /**
     *
     *
     * <h2>Gets the Type of This Shape.</h2>
     *
     * @return the shape type
     */
    ShapeType getType();

    /** Enumeration of supported shape types. */
    enum ShapeType {
        /** A 3D box defined by half-extents. */
        BOX,
        /** A sphere defined by radius. */
        SPHERE,
        /** A cylinder defined by radius and height. */
        CYLINDER,
        /** A capsule (cylinder with hemispherical caps). */
        CAPSULE,
        /** A convex hull defined by vertices. */
        CONVEX_HULL,
        /** A compound shape made of multiple child shapes. */
        COMPOUND,
        /** A static triangle mesh (for terrain/walls). */
        MESH
    }
}
