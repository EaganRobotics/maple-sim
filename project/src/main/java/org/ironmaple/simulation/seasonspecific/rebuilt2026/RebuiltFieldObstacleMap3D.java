package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.utils.FieldMirroringUtils;

public class RebuiltFieldObstacleMap3D extends SimulatedArena3D.FieldMap3D {

    private static final double FIELD_WIDTH = Inches.of(651).in(Meters);
    private static final double FIELD_HEIGHT = Inches.of(315).in(Meters);

    public RebuiltFieldObstacleMap3D() {
        super();

        // Field Perimeter Walls
        // Side walls (Length-wise): 51cm
        double sideWallHeight = 0.51;
        // Alliance walls (Width-wise): 2m
        double allianceWallHeight = 2.0;

        double wallThickness = 0.1;

        // Floor (Ground)
        // Create a large box for the floor, positioned so its top surface is at Z=0
        double floorThickness = 1.0;
        this.addBox(
                new Translation3d(FIELD_WIDTH * 1.5 / 2, FIELD_HEIGHT * 1.5 / 2, floorThickness / 2),
                new Pose3d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2, -floorThickness / 2, new Rotation3d()));

        // Top Wall (Side Wall)
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH / 2, FIELD_HEIGHT + wallThickness / 2, sideWallHeight / 2),
                new Translation3d(FIELD_WIDTH, wallThickness, sideWallHeight));

        // Bottom Wall (Side Wall)
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH / 2, -wallThickness / 2, sideWallHeight / 2),
                new Translation3d(FIELD_WIDTH, wallThickness, sideWallHeight));

        // Left Wall
        this.addBorderLine(
                new Translation3d(-wallThickness / 2, FIELD_HEIGHT / 2, allianceWallHeight / 2),
                new Translation3d(wallThickness, FIELD_HEIGHT, allianceWallHeight));

        // Right Wall
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH + wallThickness / 2, FIELD_HEIGHT / 2, allianceWallHeight / 2),
                new Translation3d(wallThickness, FIELD_HEIGHT, allianceWallHeight));

        // --- Obstacles from RebuiltFieldObstacleMap ---

        // Blue Hub
        Translation2d blueHubPos = RebuiltHub.BLUE_HUB_POS.toTranslation2d();
        addObstacle("meshes/hub.obj", new Pose3d(blueHubPos.getX(), blueHubPos.getY(), 0, new Rotation3d()));

        // Red Hub
        Translation2d redHubPos = RebuiltHub.RED_HUB_POS.toTranslation2d();
        addObstacle("meshes/hub.obj", new Pose3d(redHubPos.getX(), redHubPos.getY(), 0, new Rotation3d(0, 0, Math.PI)));

        // Blue Side Bumps
        // Blue Left (Top): 0 deg (Global Left is +Y)
        addObstacle("meshes/bump.obj", new Pose3d(4.614, 2.505, 0.0, new Rotation3d()));
        // Blue Right (Bottom): 180 deg
        addObstacle("meshes/bump.obj", new Pose3d(4.614, FIELD_HEIGHT - 2.505, 0.0, new Rotation3d(0, 0, Math.PI)));

        // Red Left (Top?): 180 deg (Global Left is +Y)
        addObstacle("meshes/bump.obj", new Pose3d(FIELD_WIDTH - 4.614, 2.505, 0.0, new Rotation3d()));
        // Red Right (Bottom?): 0 deg
        // Red Side Bumps
        addObstacle(
                "meshes/bump.obj",
                new Pose3d(FIELD_WIDTH - 4.614, FIELD_HEIGHT - 2.505, 0.0, new Rotation3d(0, 0, Math.PI)));

        // Blue Trench Fixed (-Y global relative to Left Bump)
        var trenchX = 4.614;
        var trenchFixedY = 1.427;
        var trenchHingedBaseY = 6.638;
        var trenchHingedArmY = 6.561;
        var trenchHingedArmZ = 0.619;
        addObstacle("meshes/trench_fixed.obj", new Pose3d(trenchX, trenchFixedY, 0, new Rotation3d()));

        // Blue Trench Hinged (+Y global relative to Right Bump)
        addObstacle("meshes/trench_hinged_base.obj", new Pose3d(trenchX, trenchHingedBaseY, 0, new Rotation3d()));
        addObstacle(
                "meshes/trench_hinged_arm.obj",
                new Pose3d(trenchX, trenchHingedArmY, trenchHingedArmZ, new Rotation3d()));

        // Red Trench Fixed (-Y global relative to Left Bump)
        addObstacle("meshes/trench_fixed.obj", new Pose3d(FIELD_WIDTH - trenchX, trenchFixedY, 0, new Rotation3d()));

        // Red Trench Hinged (+Y global relative to Right Bump)
        addObstacle(
                "meshes/trench_hinged_base.obj",
                new Pose3d(FIELD_WIDTH - trenchX, trenchHingedBaseY, 0, new Rotation3d()));
        addObstacle(
                "meshes/trench_hinged_arm.obj",
                new Pose3d(FIELD_WIDTH - trenchX, trenchHingedArmY, trenchHingedArmZ, new Rotation3d()));

        // TOWERS
        var blueTowerPose = new Pose3d(0.529, 3.747, 0, new Rotation3d());
        var redTowerPose = FieldMirroringUtils.flip(blueTowerPose);
        // Blue Tower (0 deg)
        addObstacle("meshes/tower.obj", blueTowerPose);
        // Red Tower (180 deg)
        addObstacle("meshes/tower.obj", redTowerPose);
    }

    public void addObstacle(String meshResourcePath, Pose3d pose) {
        getObstacles().add(new Obstacle(meshResourcePath, pose));
    }

    private void addBorderLine(Translation3d center, Translation3d size) {
        addBox(
                new Translation3d(size.getX() / 2, size.getY() / 2, size.getZ() / 2),
                new Pose3d(center, new Rotation3d()));
    }
}
