package org.firstinspires.ftc.teamcode.brainstem.follower;

import android.content.res.Resources;

import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class PathSpec {

    public enum HeadingMode {

        HOLD_START,

        HOLD,

        TANGENT
    }

    public static final class Waypoint {
        public final double x;
        public final double y;
        public final Double headingDegrees;

        public Waypoint(double x, double y) {
            this(x, y, null);
        }

        public Waypoint(double x, double y, Double headingDegrees) {
            this.x = x;
            this.y = y;
            this.headingDegrees = headingDegrees;
        }

        public static Waypoint of(double[] pose) {
            if (pose == null || pose.length < 2) {
                throw new IllegalArgumentException("pose needs x,y");
            }
            return new Waypoint(pose[0], pose[1], pose.length >= 3 ? pose[2] : null);
        }
    }

    public static final class Segment {
        public final List<Waypoint> controlPoints;
        public final HeadingMode headingMode;

        public final double maxVelocity;

        public Segment(List<Waypoint> controlPoints, HeadingMode headingMode, double maxVelocity) {
            if (controlPoints == null || controlPoints.size() < 2) {
                throw new IllegalArgumentException("Segment needs at least 2 control points");
            }
            this.controlPoints = Collections.unmodifiableList(new ArrayList<>(controlPoints));
            this.headingMode = headingMode == null ? HeadingMode.HOLD_START : headingMode;
            this.maxVelocity = maxVelocity;
        }
    }

    public final List<Segment> segments;
    public final String name;

    public PathSpec(String name, List<Segment> segments) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("PathSpec needs at least one segment");
        }
        this.name = name == null ? "" : name;
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
    }

    public PathSpec maxVelocity(double maxVelocityInPerSec) {
        List<Segment> next = new ArrayList<>(segments.size());
        for (Segment s : segments) {
            next.add(new Segment(s.controlPoints, s.headingMode, maxVelocityInPerSec));
        }
        return new PathSpec(name, next);
    }

    /** Two-point line segment. Internal builder — public factories below cover normal use. */
    private static PathSpec line(
            String name,
            double x0, double y0,
            double x1, double y1,
            HeadingMode mode,
            Double holdHeadingDeg,
            double maxVelocity) {
        List<Waypoint> pts = Arrays.asList(
                new Waypoint(x0, y0, holdHeadingDeg),
                new Waypoint(x1, y1, holdHeadingDeg));
        return new PathSpec(name, Collections.singletonList(new Segment(pts, mode, maxVelocity)));
    }

    public static PathSpec to(double[] startPose, double[] target, HeadingMode mode) {
        return to("to", startPose, target, mode, 0);
    }

    public static PathSpec to(
            String name, double[] startPose, double[] target, HeadingMode mode, double maxVelocity) {
        double x0 = startPose[0];
        double y0 = startPose[1];
        double x1 = target[0];
        double y1 = target[1];
        Double hold = target.length >= 3 ? target[2] : null;
        if (mode == HeadingMode.HOLD && hold == null) {
            hold = startPose.length >= 3 ? startPose[2] : 0.0;
        }
        return line(name, x0, y0, x1, y1, mode, hold, maxVelocity);
    }

    public static PathSpec forward(double[] startPose, double inches) {
        return forward("forward", startPose, inches);
    }

    public static PathSpec forward(String name, double[] startPose, double inches) {
        double hRad = FieldCoords.ccwRadians(Math.toRadians(startPose[2]));
        return line(
                name,
                startPose[0],
                startPose[1],
                startPose[0] + inches * Math.cos(hRad),
                startPose[1] + inches * Math.sin(hRad),
                HeadingMode.HOLD_START,
                startPose[2],
                0);
    }

    public static PathSpec strafe(double[] startPose, double inchesLeft) {
        return strafe("strafe", startPose, inchesLeft);
    }

    public static PathSpec strafe(String name, double[] startPose, double inchesLeft) {
        double hRad = FieldCoords.ccwRadians(Math.toRadians(startPose[2]));
        double lx = Math.cos(hRad + Math.PI / 2);
        double ly = Math.sin(hRad + Math.PI / 2);
        return line(
                name,
                startPose[0],
                startPose[1],
                startPose[0] + inchesLeft * lx,
                startPose[1] + inchesLeft * ly,
                HeadingMode.HOLD_START,
                startPose[2],
                0);
    }

    public static PathSpec through(String name, HeadingMode mode, double maxVelocity, double[]... poses) {
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException("through needs >= 2 poses");
        }
        List<Segment> segments = new ArrayList<>(poses.length - 1);
        for (int i = 0; i < poses.length - 1; i++) {
            Waypoint a = Waypoint.of(poses[i]);
            Waypoint b = Waypoint.of(poses[i + 1]);
            Double hold = mode == HeadingMode.HOLD
                    ? (b.headingDegrees != null ? b.headingDegrees : a.headingDegrees)
                    : a.headingDegrees;
            segments.add(new Segment(
                    Arrays.asList(
                            new Waypoint(a.x, a.y, hold),
                            new Waypoint(b.x, b.y, hold)),
                    mode,
                    maxVelocity));
        }
        return new PathSpec(name, segments);
    }

    public static PathSpec through(String name, double[]... poses) {
        return through(name, HeadingMode.HOLD_START, 0, poses);
    }

    public static PathSpec curve(String name, HeadingMode mode, Double holdHeadingDeg, Waypoint... controls) {
        return curve(name, mode, holdHeadingDeg, 0, controls);
    }

    public static PathSpec curve(
            String name, HeadingMode mode, Double holdHeadingDeg, double maxVelocity, Waypoint... controls) {
        if (controls == null || controls.length < 2) {
            throw new IllegalArgumentException("curve needs >= 2 control points");
        }
        List<Waypoint> pts = new ArrayList<>();
        for (Waypoint w : controls) {
            pts.add(new Waypoint(w.x, w.y, holdHeadingDeg != null ? holdHeadingDeg : w.headingDegrees));
        }
        return new PathSpec(name, Collections.singletonList(new Segment(pts, mode, maxVelocity)));
    }

    public static PathSpec parse(String json) {
        return PathPlannerImport.parse(json);
    }

    public static PathSpec fromRaw(Resources resources, int rawResId) {
        try (InputStream in = resources.openRawResource(rawResId)) {
            return parse(readUtf8(in));
        } catch (IOException e) {
            throw new IllegalArgumentException("Failed to read raw path resource", e);
        }
    }

    public static PathSpec fromFile(String fileName) {
        File file = resolvePathFile(fileName);
        try (InputStream in = new FileInputStream(file)) {
            return parse(readUtf8(in));
        } catch (IOException e) {
            throw new IllegalArgumentException("Failed to read path file: " + file.getAbsolutePath(), e);
        }
    }

    public String toJson() {
        try {
            JSONObject root = new JSONObject();
            root.put("format", PathPlannerImport.FORMAT_V1);
            root.put("name", name);
            root.put("units", "inches");
            root.put("origin", "center");
            JSONArray segs = new JSONArray();
            for (Segment s : segments) {
                JSONObject sj = new JSONObject();
                sj.put("headingMode", s.headingMode.name());
                sj.put("maxVelocity", s.maxVelocity);
                JSONArray cps = new JSONArray();
                for (Waypoint w : s.controlPoints) {
                    JSONObject wj = new JSONObject();
                    wj.put("x", w.x);
                    wj.put("y", w.y);
                    if (w.headingDegrees != null) {
                        wj.put("headingDegrees", w.headingDegrees);
                    }
                    cps.put(wj);
                }
                sj.put("controlPoints", cps);
                segs.put(sj);
            }
            root.put("segments", segs);
            return root.toString(2);
        } catch (JSONException e) {
            throw new IllegalStateException("PathSpec toJson failed", e);
        }
    }

    static PathSpec fromSegmentsJson(String json) {
        try {
            JSONObject root = new JSONObject(json);
            String name = root.optString("name", "");
            JSONArray segs = root.getJSONArray("segments");
            List<Segment> segments = new ArrayList<>();
            for (int i = 0; i < segs.length(); i++) {
                JSONObject sj = segs.getJSONObject(i);
                HeadingMode mode = HeadingMode.valueOf(sj.optString("headingMode", "HOLD_START"));
                double maxV = sj.optDouble("maxVelocity", 0);
                JSONArray cps = sj.getJSONArray("controlPoints");
                List<Waypoint> pts = new ArrayList<>();
                for (int j = 0; j < cps.length(); j++) {
                    JSONObject wj = cps.getJSONObject(j);
                    Double h = wj.has("headingDegrees") ? wj.getDouble("headingDegrees") : null;
                    pts.add(new Waypoint(wj.getDouble("x"), wj.getDouble("y"), h));
                }
                segments.add(new Segment(pts, mode, maxV));
            }
            return new PathSpec(name, segments);
        } catch (JSONException e) {
            throw new IllegalArgumentException("Invalid PathSpec JSON", e);
        }
    }

    private static File resolvePathFile(String fileName) {
        if (fileName == null || fileName.trim().isEmpty()) {
            throw new IllegalArgumentException("fileName required");
        }
        File direct = new File(fileName);
        if (direct.isAbsolute()) {
            return direct;
        }
        return new File(new File(android.os.Environment.getExternalStorageDirectory(), "FIRST/paths"), fileName);
    }

    private static String readUtf8(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append('\n');
            }
        }
        return sb.toString();
    }
}
