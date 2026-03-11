package org.ironmaple.utils;

import edu.wpi.first.math.geometry.Translation3d;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 *
 *
 * <h1>OBJ Mesh Loader for Collision Shapes</h1>
 *
 * <p>Parses OBJ files specifically for generating collision meshes.
 *
 * <p>Supports V-HACD style decomposition where multiple "objects" (o name) or "groups" (g name) in a single file
 * represent separate convex hulls that should be combined into a single compound shape.
 */
public class ObjLoader {

    /**
     *
     *
     * <h2>Loads Convex Hulls from an OBJ Resource.</h2>
     *
     * <p>Parses the OBJ file found at the given resource path and extracts vertices for each object/group.
     *
     * @param resourcePath the path to the OBJ file in the classpath (e.g. "meshes/field.obj")
     * @return a list of convex hulls, where each hull is an array of vertices
     * @throws IOException if the file cannot be read
     */
    public static List<Translation3d[]> loadConvexHulls(String resourcePath) throws IOException {
        try (InputStream is = ObjLoader.class.getClassLoader().getResourceAsStream(resourcePath)) {
            if (is == null) {
                throw new IOException("Resource not found: " + resourcePath);
            }
            return parseObj(is);
        }
    }

    private static List<Translation3d[]> parseObj(InputStream inputStream) throws IOException {
        List<Translation3d[]> hulls = new ArrayList<>();
        List<Translation3d> currentHullVertices = new ArrayList<>();
        // Global vertex list - OBJ indices refer to this
        List<Translation3d> allVertices = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream, StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }

                String[] tokens = line.split("\\s+");
                switch (tokens[0]) {
                    case "v":
                        // Vertex definition: v x y z
                        double x = Double.parseDouble(tokens[1]);
                        double y = Double.parseDouble(tokens[2]);
                        double z = Double.parseDouble(tokens[3]);
                        allVertices.add(new Translation3d(x, y, z));
                        break;

                    case "o":
                    case "g":
                        // New object/group - saves previous hull if it exists
                        if (!currentHullVertices.isEmpty()) {
                            hulls.add(currentHullVertices.toArray(new Translation3d[0]));
                            currentHullVertices = new ArrayList<>();
                        }
                        break;

                    case "f":
                        // Face definition: f v1 v2 v3 ...
                        // We extract vertices referenced by this face and add them to the current hull
                        for (int i = 1; i < tokens.length; i++) {
                            String vertexToken = tokens[i];
                            // Handle format v/vt/vn or v//vn
                            int slashIdx = vertexToken.indexOf('/');
                            String vertexIndexStr = slashIdx != -1 ? vertexToken.substring(0, slashIdx) : vertexToken;
                            int vertexIndex = Integer.parseInt(vertexIndexStr);

                            // OBJ is 1-indexed, handle negative indices (relative to end)
                            Translation3d vertex;
                            if (vertexIndex > 0) {
                                vertex = allVertices.get(vertexIndex - 1);
                            } else {
                                vertex = allVertices.get(allVertices.size() + vertexIndex);
                            }
                            currentHullVertices.add(vertex);
                        }
                        break;
                }
            }

            // Add the last hull
            if (!currentHullVertices.isEmpty()) {
                hulls.add(currentHullVertices.toArray(new Translation3d[0]));
            }
        }

        return hulls;
    }
}
