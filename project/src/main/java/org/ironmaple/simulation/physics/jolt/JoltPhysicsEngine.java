package org.ironmaple.simulation.physics.jolt;

import com.github.stephengold.joltjni.*;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.readonly.ConstShape;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import electrostatic4j.snaploader.LibraryInfo;
import electrostatic4j.snaploader.LoadingCriterion;
import electrostatic4j.snaploader.NativeBinaryLoader;
import electrostatic4j.snaploader.filesystem.DirectoryPath;
import electrostatic4j.snaploader.platform.NativeDynamicLibrary;
import electrostatic4j.snaploader.platform.util.PlatformPredicate;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Jolt Physics Engine Implementation</h1>
 *
 * <p>Implements the {@link PhysicsEngine} interface using Jolt Physics via the jolt-jni wrapper.
 *
 * <p>Provides high-performance 3D rigid body dynamics with internal multi-threading.
 */
public class JoltPhysicsEngine implements PhysicsEngine {
    private static final AtomicBoolean LIBRARY_LOADED = new AtomicBoolean(false);

    // Collision layers
    private static final int NUM_OBJECT_LAYERS = 3;
    public static final int OBJ_LAYER_MOVING = 0;
    public static final int OBJ_LAYER_NON_MOVING = 1;
    public static final int OBJ_LAYER_RAYCAST = 2; // Layer that ignores MOVING objects

    // Simple CSV Logger
    private static java.io.PrintWriter csvLog;

    static {
        try {
            csvLog = new java.io.PrintWriter(new java.io.FileWriter("physics_debug.csv"));
            csvLog.println("Time,Type,Details");
        } catch (java.io.IOException e) {
            e.printStackTrace();
        }
    }

    public static void log(String type, String details) {
        if (csvLog != null) {
            csvLog.println(System.currentTimeMillis() + "," + type + "," + details);
            csvLog.flush();
        }
    }

    private final JoltConfig config;
    private PhysicsSystem physicsSystem;
    private BodyInterface bodyInterface;
    private TempAllocator tempAllocator;
    private JobSystem jobSystem;

    // Collision filtering
    private ObjectLayerPairFilterTable ovoFilter;
    private BroadPhaseLayerInterfaceTable layerMap;
    private ObjectVsBroadPhaseLayerFilterTable ovbFilter;

    // Body tracking
    private final Map<Integer, JoltBody> bodiesById = new ConcurrentHashMap<>();
    private final Set<JoltBody> allBodies = ConcurrentHashMap.newKeySet();

    /** Creates a Jolt physics engine with default configuration. */
    public JoltPhysicsEngine() {
        this.config = JoltConfig.defaultConfig();
    }

    /**
     * Creates a Jolt physics engine with custom configuration.
     *
     * @param config the configuration
     */
    public JoltPhysicsEngine(JoltConfig config) {
        this.config = config;
    }

    @Override
    public void initialize() {
        if (edu.wpi.first.wpilibj.RobotBase.isReal()) {
            throw new IllegalStateException(
                    "CRITICAL: 3D Physics Engines (Jolt/Bullet) CANNOT be executed on the RoboRIO. "
                            + "They rely on un-compiled heavy native binaries and require heavy multi-threading. "
                            + "Please ensure your physics engine instantiation is wrapped in `if (RobotBase.isSimulation())`.");
        }

        loadLibrary();

        // Start auto-cleaner for native memory management
        JoltPhysicsObject.startCleaner();

        // Register allocators and callbacks
        Jolt.registerDefaultAllocator();
        Jolt.installDefaultAssertCallback();
        Jolt.installDefaultTraceCallback();

        // Create factory and register types
        boolean success = Jolt.newFactory();
        if (!success) {
            throw new RuntimeException("Failed to create Jolt factory");
        }
        Jolt.registerTypes();

        // Set up collision layers
        setupCollisionLayers();

        // Create the physics system
        physicsSystem = new PhysicsSystem();
        int numBodyMutexes = 0; // Use default
        physicsSystem.init(
                config.getMaxBodies(),
                numBodyMutexes,
                config.getMaxBodyPairs(),
                config.getMaxContacts(),
                layerMap,
                ovbFilter,
                ovoFilter);

        bodyInterface = physicsSystem.getBodyInterface();

        // Create temp allocator
        tempAllocator = new TempAllocatorMalloc();

        // Create job system with configured thread count
        jobSystem = new JobSystemThreadPool(Jolt.cMaxPhysicsJobs, Jolt.cMaxPhysicsBarriers, numWorkerThreads);

        // Set default gravity (Z-down for FRC field)
        physicsSystem.setGravity(new Vec3(0f, 0f, -9.81f));
    }

    private int numWorkerThreads = 4;

    /**
     * Sets the number of worker threads for the physics engine.
     *
     * <p>Must be called before initialize().
     *
     * @param numWorkerThreads number of threads (default 4)
     */
    public void setWorkerThreadCount(int numWorkerThreads) {
        if (physicsSystem != null) {
            throw new IllegalStateException("Cannot set worker thread count after initialization");
        }
        this.numWorkerThreads = numWorkerThreads;
    }

    /** Loads the native Jolt library. Safe to call multiple times. */
    public static void loadLibrary() {
        if (LIBRARY_LOADED.compareAndSet(false, true)) {
            LibraryInfo info = new LibraryInfo(null, "joltjni", DirectoryPath.USER_DIR);
            NativeBinaryLoader loader = new NativeBinaryLoader(info);

            NativeDynamicLibrary[] libraries = {
                new NativeDynamicLibrary("linux/aarch64/com/github/stephengold", PlatformPredicate.LINUX_ARM_64),
                new NativeDynamicLibrary("linux/armhf/com/github/stephengold", PlatformPredicate.LINUX_ARM_32),
                new NativeDynamicLibrary("linux/x86-64/com/github/stephengold", PlatformPredicate.LINUX_X86_64),
                new NativeDynamicLibrary("osx/aarch64/com/github/stephengold", PlatformPredicate.MACOS_ARM_64),
                new NativeDynamicLibrary("osx/x86-64/com/github/stephengold", PlatformPredicate.MACOS_X86_64),
                new NativeDynamicLibrary("windows/x86-64/com/github/stephengold", PlatformPredicate.WIN_X86_64)
            };
            loader.registerNativeLibraries(libraries).initPlatformLibrary();

            try {
                loader.loadLibrary(LoadingCriterion.CLEAN_EXTRACTION);
            } catch (Exception e) {
                throw new RuntimeException("Failed to load Jolt-JNI native library!", e);
            }
        }
    }

    private void setupCollisionLayers() {
        // Jolt recommends separating Static and Dynamic objects into different
        // BroadPhase layers
        // to minimize unnecessary overlap checks.
        // BP Layer 0: NON_MOVING (Static)
        // BP Layer 1: MOVING (Dynamic)
        int numBpLayers = 2;
        int BP_LAYER_NON_MOVING = 0;
        int BP_LAYER_MOVING = 1;

        ovoFilter = new ObjectLayerPairFilterTable(NUM_OBJECT_LAYERS);

        // Layer 0: MOVING (Robot)
        // Collides with MOVING (other robots) and NON_MOVING (ground/walls)
        ovoFilter.enableCollision(OBJ_LAYER_MOVING, OBJ_LAYER_MOVING);
        ovoFilter.enableCollision(OBJ_LAYER_MOVING, OBJ_LAYER_NON_MOVING);
        ovoFilter.disableCollision(OBJ_LAYER_MOVING, OBJ_LAYER_RAYCAST);

        // Layer 1: NON_MOVING (Ground/Walls)
        // Collides with MOVING (robots) and RAYCAST (rays looking for ground)
        ovoFilter.enableCollision(OBJ_LAYER_NON_MOVING, OBJ_LAYER_MOVING);
        ovoFilter.disableCollision(OBJ_LAYER_NON_MOVING, OBJ_LAYER_NON_MOVING); // Static doesn't collide with static
        ovoFilter.enableCollision(OBJ_LAYER_NON_MOVING, OBJ_LAYER_RAYCAST);

        // Layer 2: RAYCAST (Sensor Rays)
        // Collides with NON_MOVING (ground) only. Ignores MOVING (robot self).
        ovoFilter.disableCollision(OBJ_LAYER_RAYCAST, OBJ_LAYER_MOVING);
        ovoFilter.enableCollision(OBJ_LAYER_RAYCAST, OBJ_LAYER_NON_MOVING);
        ovoFilter.disableCollision(OBJ_LAYER_RAYCAST, OBJ_LAYER_RAYCAST);

        layerMap = new BroadPhaseLayerInterfaceTable(NUM_OBJECT_LAYERS, numBpLayers);

        // Map Object Layers to BroadPhase Layers
        layerMap.mapObjectToBroadPhaseLayer(OBJ_LAYER_MOVING, BP_LAYER_MOVING);
        layerMap.mapObjectToBroadPhaseLayer(OBJ_LAYER_NON_MOVING, BP_LAYER_NON_MOVING);
        // Map RAYCAST to MOVING broadphase so it uses a different broadphase than
        // ground
        // The object-layer filter will prevent it from actually colliding with MOVING
        // objects
        layerMap.mapObjectToBroadPhaseLayer(OBJ_LAYER_RAYCAST, BP_LAYER_MOVING);

        ovbFilter = new ObjectVsBroadPhaseLayerFilterTable(layerMap, numBpLayers, ovoFilter, NUM_OBJECT_LAYERS);
    }

    @Override
    public void shutdown() {
        removeAllBodies();
        physicsSystem = null;
        tempAllocator = null;
        jobSystem = null;
    }

    @Override
    public void step(Time deltaTime) {
        if (physicsSystem == null) return;
        float dt = (float) deltaTime.baseUnitMagnitude();
        int collisionSteps = 1;

        // DEBUG: Log step call
        int bodyCount = allBodies.size();
        if (bodyCount > 0) {
            // System.out.println("[Jolt] step dt=" + dt + "s, bodies=" + bodyCount);
        }

        physicsSystem.update(dt, collisionSteps, tempAllocator, jobSystem);
    }

    @Override
    public PhysicsBody createDynamicBody(PhysicsShape shape, double massKg, Pose3d initialPose) {
        // Legacy default: use typical robotics defaults
        return createDynamicBodyInternal(shape, massKg, 0.8, 0.05, 0.05, 0.05, initialPose, true);
    }

    @Override
    public PhysicsBody createDynamicBody(
            PhysicsShape shape,
            double massKg,
            double friction,
            double restitution,
            double linearDamping,
            double angularDamping,
            Pose3d initialPose) {
        return createDynamicBodyInternal(
                shape, massKg, friction, restitution, linearDamping, angularDamping, initialPose, true);
    }

    /** Creates a dynamic body without adding it to the physics system. */
    public PhysicsBody createDynamicBodyNoAdd(PhysicsShape shape, double massKg, Pose3d initialPose) {
        return createDynamicBodyInternal(shape, massKg, 0.8, 0.05, 0.05, 0.05, initialPose, false);
    }

    /** Creates a dynamic body with properties without adding it to the physics system. */
    public PhysicsBody createDynamicBodyNoAdd(
            PhysicsShape shape,
            double massKg,
            double friction,
            double restitution,
            double linearDamping,
            double angularDamping,
            Pose3d initialPose) {
        return createDynamicBodyInternal(
                shape, massKg, friction, restitution, linearDamping, angularDamping, initialPose, false);
    }

    private PhysicsBody createDynamicBodyInternal(
            PhysicsShape shape,
            double massKg,
            double friction,
            double restitution,
            double linearDamping,
            double angularDamping,
            Pose3d initialPose,
            boolean add) {
        JoltShape joltShape = (JoltShape) shape;
        ConstShape constShape = joltShape.getShape();

        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(constShape);
        bcs.setPosition(JoltBody.toRVec3(initialPose.getTranslation()));
        bcs.setRotation(JoltBody.toQuat(initialPose.getRotation()));
        bcs.setMotionType(EMotionType.Dynamic);
        bcs.setObjectLayer(OBJ_LAYER_MOVING);

        // Apply physical properties from arguments
        bcs.setFriction((float) friction);
        bcs.setRestitution((float) restitution);
        bcs.setLinearDamping((float) linearDamping);
        bcs.setAngularDamping((float) angularDamping);

        // Correct mass/inertia if mass override is provided
        if (massKg > 0) {
            MassProperties mp = constShape.getMassProperties();
            mp.scaleToMass((float) massKg);
            bcs.setMassPropertiesOverride(mp);
            bcs.setOverrideMassProperties(
                    com.github.stephengold.joltjni.enumerate.EOverrideMassProperties.MassAndInertiaProvided);
        }

        // Allow sleeping to improve performance
        bcs.setAllowSleeping(true);

        Body body = bodyInterface.createBody(bcs);
        if (body == null) {
            throw new RuntimeException("Failed to create dynamic body - max bodies reached?");
        }

        if (add) {
            bodyInterface.addBody(body, EActivation.Activate);
        }

        JoltBody joltBody = new JoltBody(body, physicsSystem, false, massKg);
        bodiesById.put(joltBody.getTrackingId(), joltBody);
        allBodies.add(joltBody);

        // Get AABB for debugging
        var aabb = body.getWorldSpaceBounds();
        var min = aabb.getMin();
        var max = aabb.getMax();
        // System.out.println("[Jolt] Created dynamic body id=" +
        // joltBody.getTrackingId() + " mass=" + massKg
        // + "kg, added=" + add + ", totalBodies=" + allBodies.size()
        // + " AABB: min=(" + String.format("%.3f", min.getX()) + ", "
        // + String.format("%.3f", min.getY()) + ", "
        // + String.format("%.3f", min.getZ()) + ") max=("
        // + String.format("%.3f", max.getX()) + ", "
        // + String.format("%.3f", max.getY()) + ", "
        // + String.format("%.3f", max.getZ()) + ")");

        return joltBody;
    }

    @Override
    public PhysicsBody createStaticBody(PhysicsShape shape, Pose3d pose) {
        return createStaticBodyInternal(shape, pose, true);
    }

    /** Creates a static body without adding it to the physics system. */
    public PhysicsBody createStaticBodyNoAdd(PhysicsShape shape, Pose3d pose) {
        return createStaticBodyInternal(shape, pose, false);
    }

    private PhysicsBody createStaticBodyInternal(PhysicsShape shape, Pose3d pose, boolean add) {
        JoltShape joltShape = (JoltShape) shape;
        ConstShape constShape = joltShape.getShape();

        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(constShape);
        bcs.setPosition(JoltBody.toRVec3(pose.getTranslation()));
        bcs.setRotation(JoltBody.toQuat(pose.getRotation()));
        bcs.setMotionType(EMotionType.Static);
        bcs.setObjectLayer(OBJ_LAYER_NON_MOVING);
        // Better defaults for robotics
        bcs.setFriction(0.8f);
        bcs.setRestitution(0.05f);

        Body body = bodyInterface.createBody(bcs);
        if (body == null) {
            throw new RuntimeException("Failed to create static body - max bodies reached?");
        }

        if (add) {
            bodyInterface.addBody(body, EActivation.DontActivate);
        }

        JoltBody joltBody = new JoltBody(body, physicsSystem, true, 0);
        bodiesById.put(joltBody.getTrackingId(), joltBody);
        allBodies.add(joltBody);

        // System.out.println("[Jolt] Created static body: trackingId=" +
        // joltBody.getTrackingId() + " joltId="
        // + joltBody.getJoltBodyId() + " added=" + add + " layer="
        // + body.getObjectLayer() + " totalBodies=" + allBodies.size());

        return joltBody;
    }

    @Override
    public PhysicsBody createKinematicBody(PhysicsShape shape, Pose3d pose) {
        JoltShape joltShape = (JoltShape) shape;
        ConstShape constShape = joltShape.getShape();

        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(constShape);
        bcs.setPosition(JoltBody.toRVec3(pose.getTranslation()));
        bcs.setRotation(JoltBody.toQuat(pose.getRotation()));
        bcs.setMotionType(EMotionType.Kinematic);
        bcs.setObjectLayer(OBJ_LAYER_MOVING);
        // Kinematic bodies should have good friction to push things properly
        bcs.setFriction(0.8f);
        bcs.setRestitution(0.0f);

        Body body = bodyInterface.createBody(bcs);
        if (body == null) {
            throw new RuntimeException("Failed to create kinematic body");
        }

        bodyInterface.addBody(body, EActivation.Activate);

        // Mass 0 for kinematic usually, or Jolt allows big mass. Jolt Kinematic implies
        // infinite mass for collision response.
        JoltBody joltBody = new JoltBody(body, physicsSystem, false, 0);
        bodiesById.put(joltBody.getTrackingId(), joltBody);
        allBodies.add(joltBody);

        return joltBody;
    }

    /** Adds an existing body to the physics space. */
    @Override
    public void addBody(PhysicsBody body) {
        if (body instanceof JoltBody joltBody) {
            // Use the actual Jolt body ID, not the tracking ID
            var joltBodyId = joltBody.getBody().getId();
            bodyInterface.addBody(joltBodyId, EActivation.Activate);
            // Ensure tracking
            bodiesById.put(joltBody.getTrackingId(), joltBody);
            allBodies.add(joltBody);

            // System.out.println("[Jolt] addBody: trackingId=" + joltBody.getTrackingId()
            // + " joltBodyId=" + joltBodyId
            // + " isStatic=" + joltBody.isStatic()
            // + " totalBodies=" + allBodies.size());
        }
    }

    @Override
    public void removeBody(PhysicsBody body) {
        if (body instanceof JoltBody joltBody) {
            // Use the actual Jolt body ID, not the tracking ID
            bodyInterface.removeBody(joltBody.getBody().getId());
            bodyInterface.destroyBody(joltBody.getBody().getId());
            bodiesById.remove(joltBody.getTrackingId());
            allBodies.remove(joltBody);
        }
    }

    @Override
    public void removeAllBodies() {
        for (JoltBody body : new ArrayList<>(allBodies)) {
            removeBody(body);
        }
    }

    @Override
    public java.util.List<PhysicsBody> getBodies() {
        return new java.util.ArrayList<>(allBodies);
    }

    @Override
    public PhysicsShape createBoxShape(Translation3d halfExtentsMeters) {
        Vec3 halfExtents = new Vec3(
                (float) halfExtentsMeters.getX(), (float) halfExtentsMeters.getY(), (float) halfExtentsMeters.getZ());
        BoxShape boxShape = new BoxShape(halfExtents);
        return new JoltShape(boxShape, PhysicsShape.ShapeType.BOX);
    }

    @Override
    public PhysicsShape createOffsetShape(PhysicsShape shape, Translation3d offset) {
        JoltShape joltShape = (JoltShape) shape;
        ConstShape baseShape = joltShape.getShape();

        Vec3 pos = JoltBody.toVec3(offset);
        Quat rot = Quat.sIdentity();

        RotatedTranslatedShapeSettings settings = new RotatedTranslatedShapeSettings(pos, rot, baseShape);
        ShapeResult result = settings.create();
        if (result.hasError()) throw new RuntimeException("Failed to create offset shape: " + result.getError());
        return new JoltShape(result.get(), PhysicsShape.ShapeType.COMPOUND);
    }

    @Override
    public PhysicsShape createSphereShape(double radiusMeters) {
        SphereShape sphereShape = new SphereShape((float) radiusMeters);
        return new JoltShape(sphereShape, PhysicsShape.ShapeType.SPHERE);
    }

    @Override
    public PhysicsShape createCylinderShape(double radiusMeters, double heightMeters) {
        float halfHeight = (float) (heightMeters / 2.0);
        CylinderShape cylinderShape = new CylinderShape(halfHeight, (float) radiusMeters);
        return new JoltShape(cylinderShape, PhysicsShape.ShapeType.CYLINDER);
    }

    @Override
    public PhysicsShape createConvexHullShape(Translation3d[] vertices) {
        if (vertices.length < 3) {
            throw new IllegalArgumentException("Convex hull must have at least 3 vertices");
        }

        // Build direct float buffer from vertices for JNI safety
        java.nio.ByteBuffer byteBuf = java.nio.ByteBuffer.allocateDirect(vertices.length * 3 * 4);
        byteBuf.order(java.nio.ByteOrder.nativeOrder());
        java.nio.FloatBuffer buffer = byteBuf.asFloatBuffer();

        for (Translation3d v : vertices) {
            buffer.put((float) v.getX());
            buffer.put((float) v.getY());
            buffer.put((float) v.getZ());
        }
        buffer.flip();

        ConvexHullShapeSettings settings = new ConvexHullShapeSettings(vertices.length, buffer);
        ConstShape shape = settings.create().get();
        return new JoltShape(shape, PhysicsShape.ShapeType.CONVEX_HULL);
    }

    @Override
    public PhysicsShape createCompoundShapeFromMesh(String resourcePath) {
        try {
            List<Translation3d[]> hulls = org.ironmaple.utils.ObjLoader.loadConvexHulls(resourcePath);

            System.out.println("[Jolt] Loaded mesh " + resourcePath + " with " + hulls.size() + " hulls");

            if (hulls.isEmpty()) {
                throw new RuntimeException("No hulls found in OBJ file: " + resourcePath);
            }

            if (hulls.size() == 1) {
                return createConvexHullShape(hulls.get(0));
            }

            // Create compound shape from multiple convex hulls
            StaticCompoundShapeSettings compound = new StaticCompoundShapeSettings();
            int validHulls = 0;

            for (int i = 0; i < hulls.size(); i++) {
                Translation3d[] verts = hulls.get(i);
                if (verts.length < 3) {
                    System.err.println("[Jolt] Warning: Skipping degenerate hull " + i + " in " + resourcePath
                            + " with " + verts.length + " vertices");
                    continue;
                }

                // Use Direct Buffer for JNI safety
                java.nio.ByteBuffer byteBuf = java.nio.ByteBuffer.allocateDirect(verts.length * 3 * 4);
                byteBuf.order(java.nio.ByteOrder.nativeOrder());
                java.nio.FloatBuffer buffer = byteBuf.asFloatBuffer();

                for (Translation3d v : verts) {
                    buffer.put((float) v.getX());
                    buffer.put((float) v.getY());
                    buffer.put((float) v.getZ());
                }
                buffer.flip();

                try {
                    ConvexHullShapeSettings hullSettings = new ConvexHullShapeSettings(verts.length, buffer);
                    ConstShape hull = hullSettings.create().get();
                    compound.addShape(new Vec3(0, 0, 0), Quat.sIdentity(), hull);
                    validHulls++;
                } catch (Exception e) {
                    System.err.println("[Jolt] Failed to create hull shape for " + resourcePath + " hull " + i);
                    e.printStackTrace();
                }
            }

            if (validHulls == 0) {
                throw new RuntimeException("No valid hulls created for mesh: " + resourcePath);
            }

            ConstShape compoundShape = compound.create().get();
            return new JoltShape(compoundShape, PhysicsShape.ShapeType.COMPOUND);
        } catch (java.io.IOException e) {
            throw new RuntimeException("Failed to load mesh: " + resourcePath, e);
        }
    }

    @Override
    public Optional<RaycastResult> raycast(Translation3d origin, Translation3d direction, double maxDistance) {
        return raycast(origin, direction, maxDistance, null);
    }

    @Override
    public Optional<RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance, PhysicsBody excludeBody) {
        if (physicsSystem == null) return Optional.empty();

        // Normalize direction
        double len = Math.sqrt(direction.getX() * direction.getX()
                + direction.getY() * direction.getY()
                + direction.getZ() * direction.getZ());
        if (len < 1e-9) return Optional.empty();
        Translation3d dir = new Translation3d(direction.getX() / len, direction.getY() / len, direction.getZ() / len);

        // Get the Jolt body ID to exclude (if any)
        int excludeJoltId = -1;
        if (excludeBody instanceof JoltBody) {
            excludeJoltId = ((JoltBody) excludeBody).getJoltBodyId();
        }

        // Create ray: RRayCast takes start as RVec3 and direction*distance as Vec3
        Vec3 dirVec = JoltBody.toVec3(dir.times(maxDistance));
        RRayCast ray = new RRayCast(JoltBody.toRVec3(origin), dirVec);

        // Cast ray and check results - we'll iterate to skip excluded body
        // Max 15 iterations to prevent infinite loops (increased from 5 for complex
        // bodies)
        for (int attempt = 0; attempt < 15; attempt++) {
            RayCastResult result = new RayCastResult();

            // Use OBJ_LAYER_MOVING filters which we know interact with both Static and
            // Dynamic bodies.
            // This ensures we can hit the ground.
            // The robot (also MOVING) will be hit, but our loop below will reliably skip
            // it.
            boolean hit = physicsSystem
                    .getNarrowPhaseQuery()
                    .castRay(
                            ray,
                            result,
                            physicsSystem.getDefaultBroadPhaseLayerFilter(OBJ_LAYER_MOVING),
                            physicsSystem.getDefaultLayerFilter(OBJ_LAYER_RAYCAST));
            if (!hit) {
                // No hit at all
                if (Math.random() < 0.02) {
                    // System.out.println("[Jolt] Raycast miss: originZ=" + String.format("%.3f",
                    // origin.getZ())
                    // + " attempt=" + attempt);
                }
                return Optional.empty();
            }

            int hitBodyId = result.getBodyId();
            float fraction = result.getFraction();

            // Check if we hit the excluded body
            if (hitBodyId == excludeJoltId) {
                // We hit ourselves - need to continue the ray past this point
                // Move origin past the hit point and reduce max distance
                double hitDistance = maxDistance * fraction;
                double epsilon = 0.01; // 1cm offset to get past the complex geometry/internal faces

                if (hitDistance + epsilon >= maxDistance) {
                    // No room left to continue
                    if (Math.random() < 0.02) {
                        // System.out.println("[Jolt] Raycast hit excluded body, no room to continue");
                    }
                    return Optional.empty();
                }

                // Update ray to start past the excluded body
                Translation3d newOrigin = origin.plus(dir.times(hitDistance + epsilon));
                double newMaxDistance = maxDistance - hitDistance - epsilon;

                if (newMaxDistance < 0.001) {
                    return Optional.empty();
                }

                // Create new ray for next iteration
                dirVec = JoltBody.toVec3(dir.times(newMaxDistance));
                ray = new RRayCast(JoltBody.toRVec3(newOrigin), dirVec);
                origin = newOrigin;
                maxDistance = newMaxDistance;

                if (Math.random() < 0.02) {
                    // System.out.println("[Jolt] Raycast skipping excluded body, continuing from
                    // z="
                    // + String.format("%.3f", newOrigin.getZ()));
                }
                continue;
            }

            // We hit something other than the excluded body - success!
            // Find our wrapper for the hit body
            JoltBody hitBody = null;
            for (JoltBody body : allBodies) {
                if (body.getJoltBodyId() == hitBodyId) {
                    hitBody = body;
                    break;
                }
            }

            Translation3d hitPoint = origin.plus(dir.times(maxDistance * fraction));
            Translation3d hitNormal = new Translation3d(0, 0, 1); // Default up normal

            if (Math.random() < 0.02) {
                // System.out.println("[Jolt] Raycast HIT: bodyId=" + hitBodyId + " isStatic="
                // + (hitBody != null ? hitBody.isStatic() : "unknown")
                // + " hitZ=" + String.format("%.3f", hitPoint.getZ())
                // + " fraction=" + String.format("%.3f", fraction));
            }

            return Optional.of(new RaycastResult(hitPoint, hitNormal, maxDistance * fraction, hitBody));
        }

        // Exhausted attempts
        if (Math.random() < 0.1) {
            // System.err.println("[Jolt] Raycast exhausted 15 attempts trying to skip body
            // " + excludeJoltId);
        }
        return Optional.empty();
    }

    @Override
    public void setGravity(Translation3d gravityMPS2) {
        if (physicsSystem != null) {
            physicsSystem.setGravity(JoltBody.toVec3(gravityMPS2));
        }
    }

    @Override
    public List<PhysicsBody> getOverlappingBodies(PhysicsShape shape, Pose3d pose) {
        List<PhysicsBody> overlapping = new ArrayList<>();
        if (physicsSystem == null) return overlapping;

        // Simple bounding sphere check for now
        // More advanced shape cast can be added if needed
        Translation3d center = pose.getTranslation();
        double checkRadius = 2.0; // Conservative radius

        for (JoltBody body : allBodies) {
            Translation3d bodyPos = body.getPose3d().getTranslation();
            double dist = center.getDistance(bodyPos);
            if (dist < checkRadius) {
                overlapping.add(body);
            }
        }

        return overlapping;
    }

    /** Gets the physics system for advanced access. */
    public PhysicsSystem getPhysicsSystem() {
        return physicsSystem;
    }

    /** Gets the body interface for advanced access. */
    public BodyInterface getBodyInterface() {
        return bodyInterface;
    }

    // ========== Conversion Utilities ==========

    public static Vec3 toVec3(Translation3d t) {
        return JoltBody.toVec3(t);
    }

    public static Translation3d toTranslation3d(Vec3 v) {
        return JoltBody.toTranslation3d(v);
    }

    public static Quat toQuat(edu.wpi.first.math.geometry.Rotation3d r) {
        return JoltBody.toQuat(r);
    }

    public static edu.wpi.first.math.geometry.Rotation3d toRotation3d(Quat q) {
        return JoltBody.toRotation3d(q);
    }
}
