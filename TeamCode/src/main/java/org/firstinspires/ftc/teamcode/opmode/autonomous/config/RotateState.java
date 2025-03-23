package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometry.OdometryProvider;

public class RotateState {

    public static final double ROTATE_FUDGE_FACTOR = 2d;

    public final double x;

    public final double y;

    public final double h;

    public final RotationDirection direction;

    public final double yawError;

    public final double targetHeading;

    public final boolean isHeadingWithinRange;

    public static double calculateRotationDelta(double currentHeading, double targetHeading) {
        double rawDelta = Math.abs(currentHeading - targetHeading);
        double modDelta = rawDelta % 360.0;
        return (modDelta > 180.0 ? 360.0 - modDelta : modDelta);
    }

    public RotateState(OdometryProvider odometryProvider, double targetHeading, RotationDirection direction) {
        Pose2D pos = odometryProvider.getPosition();
        x = pos.getX(odometryProvider.getDistanceUnit());
        y = pos.getY(odometryProvider.getDistanceUnit());
        h = pos.getHeading(odometryProvider.getAngleUnit());
        this.direction = direction;
        this.targetHeading = targetHeading;
        yawError = calculateRotationDelta(this.h, targetHeading);
        isHeadingWithinRange = (Math.abs(yawError) <= ROTATE_FUDGE_FACTOR);
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Action: ", "ROTATE");
        telemetry.addData("Pos X: ", this.x);
        telemetry.addData("Pos Y: ", this.y);
        telemetry.addData("Pox H: ", this.h);
        telemetry.addData("Yaw Error: ", this.yawError);
        telemetry.addData("Direction: ", this.direction.name());
        telemetry.addData("Target Heading: ", this.targetHeading);
        telemetry.addData("Rotate Complete: ", this.isHeadingWithinRange);
        telemetry.update();
    }

}
