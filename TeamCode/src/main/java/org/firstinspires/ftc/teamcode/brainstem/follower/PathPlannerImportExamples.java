package org.firstinspires.ftc.teamcode.brainstem.follower;

public final class PathPlannerImportExamples {

    private PathPlannerImportExamples() {}

    public static void runSelfCheck() {
        PathSpec cubic = PathSpec.parse("{"
                + "\"format\":\"brainstem.path.v1\","
                + "\"name\":\"cubic\","
                + "\"units\":\"inches\","
                + "\"origin\":\"center\","
                + "\"waypoints\":["
                + "{\"x\":0,\"y\":0,\"headingDegrees\":0,"
                + "\"outgoing\":{\"x\":10,\"y\":0},\"maxLinearSpeed\":0},"
                + "{\"x\":24,\"y\":24,\"headingDegrees\":45,"
                + "\"incoming\":{\"x\":24,\"y\":10},\"maxLinearSpeed\":30,\"headingMode\":\"HOLD\"}"
                + "]}");
        assertTrue("name", "cubic".equals(cubic.name));
        assertTrue("one segment", cubic.segments.size() == 1);
        assertTrue("cubic has 4 controls", cubic.segments.get(0).controlPoints.size() == 4);
        assertNear("cap 30", cubic.segments.get(0).maxVelocity, 30, 1e-6);
        assertTrue("HOLD", cubic.segments.get(0).headingMode == PathSpec.HeadingMode.HOLD);

        PathSpec meters = PathSpec.parse("{"
                + "\"units\":\"meters\",\"origin\":\"center\","
                + "\"waypoints\":["
                + "{\"x\":0,\"y\":0},"
                + "{\"x\":1,\"y\":0,\"maxLinearSpeed\":1}"
                + "]}");
        assertNear("1m → inches", meters.segments.get(0).controlPoints.get(1).x, 39.37007874, 0.01);
        assertNear("1 m/s → in/s", meters.segments.get(0).maxVelocity, 39.37007874, 0.01);

        PathSpec corner = PathSpec.parse("{"
                + "\"units\":\"inches\",\"origin\":\"corner\","
                + "\"waypoints\":["
                + "{\"x\":72,\"y\":72},"
                + "{\"x\":96,\"y\":72}"
                + "]}");
        assertNear("corner→center x0", corner.segments.get(0).controlPoints.get(0).x, 0, 1e-6);
        assertNear("corner→center y0", corner.segments.get(0).controlPoints.get(0).y, 0, 1e-6);
        assertNear("corner→center x1", corner.segments.get(0).controlPoints.get(1).x, 24, 1e-6);

        PathSpec nativeSeg = PathSpec.parse("{"
                + "\"name\":\"line\","
                + "\"segments\":[{"
                + "\"headingMode\":\"TANGENT\",\"maxVelocity\":0,"
                + "\"controlPoints\":[{\"x\":0,\"y\":0},{\"x\":12,\"y\":0}]"
                + "}]}");
        assertTrue("native segments", nativeSeg.segments.size() == 1);
        assertTrue("line 2 pts", nativeSeg.segments.get(0).controlPoints.size() == 2);
    }

    private static void assertTrue(String label, boolean ok) {
        if (!ok) {
            throw new AssertionError("PathPlannerImportExamples: " + label);
        }
    }

    private static void assertNear(String label, double actual, double expected, double tol) {
        if (Math.abs(actual - expected) > tol) {
            throw new AssertionError("PathPlannerImportExamples: " + label
                    + " expected " + expected + " got " + actual);
        }
    }
}
