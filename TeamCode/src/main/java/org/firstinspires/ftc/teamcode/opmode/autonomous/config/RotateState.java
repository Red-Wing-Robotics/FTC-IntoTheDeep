package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public RotateState(SparkFunOTOS.Pose2D currentPos, double targetHeading, RotationDirection direction) {
        x = currentPos.x;
        y = currentPos.y;
        h = currentPos.h;
        this.direction = direction;
        this.targetHeading = targetHeading;
        yawError = calculateRotationDelta(currentPos.h, targetHeading);
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
