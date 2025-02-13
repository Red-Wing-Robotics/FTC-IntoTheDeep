package org.firstinspires.ftc.teamcode.util;

// Borrowed from pedro pathing

import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotationDirection;

public class MathUtils {

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians;
        while (angle < 0) angle += 2 * Math.PI;
        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /**
     * This gets the direction to turn between a start heading and an end heading. Positive is left
     * and negative is right. This operates in radians.
     *
     * @return returns the turn direction.
     */
    public static boolean shouldTurnClockwiseRadians(double startHeading, double endHeading) {
        return !(normalizeAngle(endHeading - startHeading) >= 0 && normalizeAngle(endHeading - startHeading) <= Math.PI);
    }

    public static RotationDirection getClosestRotationDirectionDegrees(double startHeading, double endHeading) {
        boolean isClockwise = shouldTurnClockwiseRadians(Math.toRadians(startHeading), Math.toRadians(endHeading));
        return isClockwise ? RotationDirection.CLOCKWISE : RotationDirection.COUNTER_CLOCKWISE;
    }
    
}
