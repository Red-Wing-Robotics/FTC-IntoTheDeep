package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveToDistanceState {

    public static final double DRIVE_DISTANCE_FUDGE = 5d;

    public final double currentDistance;

    public final double targetDistance;

    public final double distanceError;

    public final boolean isDistanceWithinRange;

    public DriveToDistanceState(double currentDistance, double targetDistance) {
        this.currentDistance = currentDistance;
        this.targetDistance = targetDistance;
        this.distanceError = currentDistance - targetDistance;
        isDistanceWithinRange = (Math.abs(this.distanceError) <= DRIVE_DISTANCE_FUDGE);
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Action: ", "DRIVE");
        telemetry.addData("Current Distance: ", this.currentDistance);
        telemetry.addData("Target Distance: ", this.targetDistance);
        telemetry.addData("Distance Error: ", this.distanceError);
        telemetry.addData("Distance Within Range: ", this.isDistanceWithinRange);
        telemetry.update();
    }

}
