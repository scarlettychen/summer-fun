package org.firstinspires.ftc.teamcode.brainstem.follower;

import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

final class PathPlannerImport {

    static final String FORMAT_V1 = "brainstem.path.v1";

    private static final double METERS_TO_INCHES = 39.37007874015748;
    private static final double FIELD_HALF_IN = FieldCoords.WALL;

    private PathPlannerImport() {}

    public static PathSpec parse(String json) {
        if (json == null || json.trim().isEmpty()) {
            throw new IllegalArgumentException("Path JSON is empty");
        }
        String trimmed = json.trim();
        try {
            if (trimmed.startsWith("[")) {
                return fromWaypoints(new JSONArray(trimmed), "imported", "inches", "center");
            }
            JSONObject root = new JSONObject(trimmed);
            if (root.has("segments")) {
                return PathSpec.fromSegmentsJson(trimmed);
            }
            if (root.has("path") && root.get("path") instanceof JSONObject) {
                JSONObject path = root.getJSONObject("path");
                if (path.has("waypoints")) {
                    return fromWaypointObject(mergeMeta(root, path));
                }
                if (path.has("segments")) {
                    return PathSpec.fromSegmentsJson(path.toString());
                }
            }
            if (root.has("waypoints")) {
                return fromWaypointObject(root);
            }
            throw new IllegalArgumentException(
                    "Unrecognized path JSON: need segments or waypoints");
        } catch (JSONException e) {
            throw new IllegalArgumentException("Invalid path JSON", e);
        }
    }

    private static JSONObject mergeMeta(JSONObject outer, JSONObject path) throws JSONException {
        JSONObject merged = new JSONObject(path.toString());
        if (!merged.has("name") && outer.has("name")) {
            merged.put("name", outer.get("name"));
        }
        if (!merged.has("units") && outer.has("units")) {
            merged.put("units", outer.get("units"));
        }
        if (!merged.has("origin") && outer.has("origin")) {
            merged.put("origin", outer.get("origin"));
        }
        if (!merged.has("format") && outer.has("format")) {
            merged.put("format", outer.get("format"));
        }
        return merged;
    }

    private static PathSpec fromWaypointObject(JSONObject root) throws JSONException {
        String name = root.optString("name", "imported");
        String units = root.optString("units", "inches");
        String origin = root.optString("origin", "center");
        JSONArray waypoints = root.getJSONArray("waypoints");
        return fromWaypoints(waypoints, name, units, origin);
    }

    public static PathSpec fromWaypoints(
            JSONArray waypoints, String name, String units, String origin) throws JSONException {
        if (waypoints == null || waypoints.length() < 2) {
            throw new IllegalArgumentException("Need at least 2 waypoints");
        }

        List<PlannerWaypoint> pts = new ArrayList<>(waypoints.length());
        for (int i = 0; i < waypoints.length(); i++) {
            pts.add(readWaypoint(waypoints.getJSONObject(i), units, origin));
        }

        List<PathSpec.Segment> segments = new ArrayList<>();
        for (int i = 0; i < pts.size() - 1; i++) {
            PlannerWaypoint a = pts.get(i);
            PlannerWaypoint b = pts.get(i + 1);
            List<PathSpec.Waypoint> controls = new ArrayList<>();
            controls.add(a.toPathWaypoint());

            boolean hasOut = a.outX != null && a.outY != null;
            boolean hasIn = b.inX != null && b.inY != null;
            if (hasOut && hasIn) {
                controls.add(new PathSpec.Waypoint(a.outX, a.outY, null));
                controls.add(new PathSpec.Waypoint(b.inX, b.inY, null));
            } else if (hasOut) {
                controls.add(new PathSpec.Waypoint(a.outX, a.outY, null));
            } else if (hasIn) {
                controls.add(new PathSpec.Waypoint(b.inX, b.inY, null));
            }

            controls.add(b.toPathWaypoint());

            PathSpec.HeadingMode mode = b.headingMode != null ? b.headingMode : a.headingMode;
            if (mode == null) {
                mode = b.headingDegrees != null
                        ? PathSpec.HeadingMode.HOLD
                        : PathSpec.HeadingMode.HOLD_START;
            }

            double maxV = b.maxLinearSpeedInPerSec;
            if (maxV <= 0) {
                maxV = a.maxLinearSpeedInPerSec;
            }
            segments.add(new PathSpec.Segment(controls, mode, Math.max(0, maxV)));
        }

        return new PathSpec(name == null ? "imported" : name, segments);
    }

    private static PlannerWaypoint readWaypoint(JSONObject w, String units, String origin)
            throws JSONException {
        PlannerWaypoint pw = new PlannerWaypoint();
        double x = firstDouble(w, "x", "X");
        double y = firstDouble(w, "y", "Y");
        double[] xy = toFieldInches(x, y, units, origin);
        pw.x = xy[0];
        pw.y = xy[1];

        if (w.has("headingDegrees")) {
            pw.headingDegrees = w.getDouble("headingDegrees");
        } else if (w.has("heading")) {
            pw.headingDegrees = w.getDouble("heading");
        } else if (w.has("h")) {
            pw.headingDegrees = w.getDouble("h");
        }

        JSONObject outgoing = firstObject(w, "outgoing", "controlNext", "nextHandle", "handleOut");
        if (outgoing != null) {
            double[] h = handleToField(outgoing, pw.x, pw.y, units, origin, w.optBoolean("handlesRelative", false));
            pw.outX = h[0];
            pw.outY = h[1];
        }
        JSONObject incoming = firstObject(w, "incoming", "controlPrev", "prevHandle", "handleIn");
        if (incoming != null) {
            double[] h = handleToField(incoming, pw.x, pw.y, units, origin, w.optBoolean("handlesRelative", false));
            pw.inX = h[0];
            pw.inY = h[1];
        }

        double speed = firstOptionalDouble(w, Double.NaN,
                "maxLinearSpeed", "maxVelocity", "maxSpeed", "velocity");
        if (!Double.isNaN(speed) && speed > 0) {
            pw.maxLinearSpeedInPerSec = toInches(speed, units);
        }

        String mode = firstOptionalString(w, null, "headingMode", "headingInterpolation");
        if (mode != null && !mode.isEmpty()) {
            pw.headingMode = PathSpec.HeadingMode.valueOf(mode.trim().toUpperCase());
        } else if (w.optBoolean("tangentHeading", false) || w.optBoolean("followTangent", false)) {
            pw.headingMode = PathSpec.HeadingMode.TANGENT;
        }

        pw.passThrough = w.optBoolean("passPosition", w.optBoolean("passThrough", true));

        return pw;
    }

    private static double[] handleToField(
            JSONObject handle,
            double anchorXIn,
            double anchorYIn,
            String units,
            String origin,
            boolean relativeHint) throws JSONException {
        double hx = firstDouble(handle, "x", "X", "dx");
        double hy = firstDouble(handle, "y", "Y", "dy");
        boolean relative = relativeHint
                || handle.optBoolean("relative", false)
                || handle.has("dx")
                || handle.has("dy");
        if (relative) {
            double dx = toInches(hx, units);
            double dy = toInches(hy, units);
            return new double[]{anchorXIn + dx, anchorYIn + dy};
        }
        return toFieldInches(hx, hy, units, origin);
    }

    private static double[] toFieldInches(double x, double y, String units, String origin) {
        double xi = toInches(x, units);
        double yi = toInches(y, units);
        String o = origin == null ? "center" : origin.trim().toLowerCase();
        switch (o) {
            case "corner":
            case "bottom_left":
            case "bl":

                return new double[]{xi - FIELD_HALF_IN, yi - FIELD_HALF_IN};
            case "center":
            case "fieldcoords":
            default:
                return new double[]{xi, yi};
        }
    }

    private static double toInches(double value, String units) {
        if (units == null) {
            return value;
        }
        switch (units.trim().toLowerCase()) {
            case "m":
            case "meter":
            case "meters":
                return value * METERS_TO_INCHES;
            case "cm":
            case "centimeter":
            case "centimeters":
                return value * METERS_TO_INCHES / 100.0;
            case "in":
            case "inch":
            case "inches":
            default:
                return value;
        }
    }

    private static double firstDouble(JSONObject o, String... keys) throws JSONException {
        for (String k : keys) {
            if (o.has(k) && !o.isNull(k)) {
                return o.getDouble(k);
            }
        }
        throw new JSONException("Missing numeric field among " + String.join("/", keys));
    }

    private static double firstOptionalDouble(JSONObject o, double fallback, String... keys) {
        for (String k : keys) {
            if (o.has(k) && !o.isNull(k)) {
                return o.optDouble(k, fallback);
            }
        }
        return fallback;
    }

    private static String firstOptionalString(JSONObject o, String fallback, String... keys) {
        for (String k : keys) {
            if (o.has(k) && !o.isNull(k)) {
                return o.optString(k, fallback);
            }
        }
        return fallback;
    }

    private static JSONObject firstObject(JSONObject o, String... keys) {
        for (String k : keys) {
            if (o.has(k) && !o.isNull(k)) {
                Object v = o.opt(k);
                if (v instanceof JSONObject) {
                    return (JSONObject) v;
                }
            }
        }
        return null;
    }

    private static final class PlannerWaypoint {
        double x;
        double y;
        Double headingDegrees;
        Double outX;
        Double outY;
        Double inX;
        Double inY;
        double maxLinearSpeedInPerSec;
        PathSpec.HeadingMode headingMode;
        @SuppressWarnings("unused")
        boolean passThrough = true;

        PathSpec.Waypoint toPathWaypoint() {
            return new PathSpec.Waypoint(x, y, headingDegrees);
        }
    }
}
