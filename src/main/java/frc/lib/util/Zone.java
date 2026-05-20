/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a named polygon zone on the field defined by a list of (x, y) vertices.
 *
 * <p>The polygon is loaded from {@code deploy/zones/<name>.json} and automatically reloaded
 * whenever the file is edited on disk — useful for tuning zones during simulation without
 * redeploying. If a reload produces an invalid polygon (fewer than 3 points, duplicate consecutive
 * vertices, or zero area), the previous valid polygon is kept.
 *
 * <p>Vertices are logged to AdvantageKit at {@code "Zones/<name>/Vertices"} on every query for
 * visualisation in AdvantageScope.
 *
 * <p><b>JSON format:</b>
 *
 * <pre>{@code
 * {
 *   "name": "ScoringZone",
 *   "points": [
 *     { "x": 1.0, "y": 1.0 },
 *     { "x": 3.0, "y": 1.0 },
 *     { "x": 3.0, "y": 4.0 },
 *     { "x": 1.0, "y": 4.0 }
 *   ]
 * }
 * }</pre>
 *
 * <p><b>Example usage:</b>
 *
 * <pre>{@code
 * Zone scoringZone = new Zone("ScoringZone");
 * LoggedTrigger inZone = new LoggedTrigger("Zones/ScoringZone/Active", () -> scoringZone.isPoseInZone(drive.getPose()));
 * inZone.onTrue(Commands.print("Entered scoring zone!"));
 * }</pre>
 */
public class Zone {

    private static final double MIN_AREA = 1e-6;

    @Getter private final String name;
    private final File jsonFile;

    @Getter private List<Translation2d> polygon = new ArrayList<>();
    private long lastModifiedTimestamp = -1L;

    /**
     * Creates a Zone backed by {@code deploy/zones/<name>.json} and performs an initial load.
     *
     * @param name The zone name, matching the JSON filename stem.
     */
    public Zone(String name) {
        this.name = name;
        this.jsonFile = new File(Filesystem.getDeployDirectory(), "zones/" + name + ".json");

        if (!jsonFile.exists()) {
            throw new IllegalArgumentException(
                    "[Zone] No zone file found for '"
                            + name
                            + "'. Expected: "
                            + jsonFile.getAbsolutePath());
        }

        loadFromFile();
    }

    /**
     * Returns {@code true} if the robot's (x, y) position is inside the zone.
     *
     * <p>Polls the JSON file for changes and logs vertices on each call.
     */
    public boolean isPoseInZone(Pose2d pose) {
        pollAndLog();
        return pointInPolygon(pose.getTranslation());
    }

    /**
     * Returns {@code true} if the given point is inside the zone.
     *
     * <p>Polls the JSON file for changes and logs vertices on each call.
     */
    public boolean isTranslationInZone(Translation2d point) {
        pollAndLog();
        return pointInPolygon(point);
    }

    /** Checks for file changes, reloads the polygon if needed, and logs vertices. */
    private void pollAndLog() {
        if (jsonFile.exists() && jsonFile.lastModified() != lastModifiedTimestamp) {
            loadFromFile();
        }
        logPolygon();
    }

    private void loadFromFile() {
        try {
            JsonNode root = new ObjectMapper().readTree(jsonFile);
            JsonNode pointsArray = root.get("points");

            if (pointsArray == null || !pointsArray.isArray()) {
                System.err.println(
                        "[Zone '" + name + "'] Missing 'points' array — keeping previous polygon.");
                return;
            }

            List<Translation2d> candidate = new ArrayList<>();
            for (JsonNode node : pointsArray) {
                JsonNode xNode = node.get("x");
                JsonNode yNode = node.get("y");
                if (xNode == null || yNode == null) {
                    System.err.println(
                            "[Zone '"
                                    + name
                                    + "'] Point missing 'x' or 'y' field — keeping previous"
                                    + " polygon.");
                    return;
                }
                candidate.add(new Translation2d(xNode.asDouble(), yNode.asDouble()));
            }

            String validationError = validatePolygon(candidate);
            if (validationError != null) {
                System.err.println(
                        "[Zone '"
                                + name
                                + "'] Invalid polygon ("
                                + validationError
                                + ") — keeping previous polygon.");
                return;
            }

            polygon = candidate;
            lastModifiedTimestamp = jsonFile.lastModified();
            System.out.println("[Zone '" + name + "'] Loaded " + polygon.size() + " vertices.");

        } catch (IOException e) {
            System.err.println("[Zone '" + name + "'] Failed to parse file: " + e.getMessage());
        }
    }

    /**
     * Validates a candidate polygon. Returns a human-readable error string if invalid, or {@code
     * null} if the polygon is acceptable.
     *
     * <p>Checks performed:
     *
     * <ul>
     *   <li>At least 3 vertices.
     *   <li>No two consecutive vertices (including last→first) are identical.
     *   <li>Non-zero area via the shoelace formula (catches all-collinear cases).
     * </ul>
     */
    private String validatePolygon(List<Translation2d> pts) {
        if (pts.size() < 3) {
            return "fewer than 3 vertices (" + pts.size() + ")";
        }

        int n = pts.size();
        for (int i = 0; i < n; i++) {
            Translation2d a = pts.get(i);
            Translation2d b = pts.get((i + 1) % n);
            if (a.getDistance(b) < 1e-9) {
                return "duplicate consecutive vertices at index " + i;
            }
        }

        // Shoelace formula — returns zero area for collinear / degenerate polygons
        double area = 0.0;
        for (int i = 0; i < n; i++) {
            Translation2d a = pts.get(i);
            Translation2d b = pts.get((i + 1) % n);
            area += a.getX() * b.getY() - b.getX() * a.getY();
        }
        if (Math.abs(area) / 2.0 < MIN_AREA) {
            return "polygon has effectively zero area (all points may be collinear)";
        }

        return null; // valid
    }

    private boolean pointInPolygon(Translation2d point) {
        if (polygon.size() < 3) return false;

        Path2D path = new Path2D.Double();
        path.moveTo(polygon.get(0).getX(), polygon.get(0).getY());
        for (int i = 1; i < polygon.size(); i++) {
            path.lineTo(polygon.get(i).getX(), polygon.get(i).getY());
        }
        path.closePath();

        return path.contains(new Point2D.Double(point.getX(), point.getY()));
    }

    /** Logs vertices as {@link Translation3d} (z = 0) closing the loop for AdvantageScope. */
    private void logPolygon() {
        int n = polygon.size();
        // Include a closing vertex so AdvantageScope draws a complete outline
        Translation3d[] vertices = new Translation3d[n == 0 ? 0 : n + 1];
        for (int i = 0; i < n; i++) {
            Translation2d v = polygon.get(i);
            vertices[i] = new Translation3d(v.getX(), v.getY(), 0.0);
        }
        if (n > 0) {
            Translation2d first = polygon.get(0);
            vertices[n] = new Translation3d(first.getX(), first.getY(), 0.0);
        }
        Logger.recordOutput("Zones/" + name + "/Vertices", vertices);
    }
}
