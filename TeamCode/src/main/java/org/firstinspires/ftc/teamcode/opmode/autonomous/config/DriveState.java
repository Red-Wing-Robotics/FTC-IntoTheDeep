package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveState {

    public static final double DRIVE_POSITION_FUDGE = 1d;

    public final double x;

    public final double y;

    public final double h;

    public final double xError;

    public final double yError;

    public final double targetX;

    public final double targetY;

    public final boolean isDriveWithinRange;

    public DriveState(SparkFunOTOS.Pose2D currentPos, double targetX, double targetY) {
        x = currentPos.x;
        y = currentPos.y;
        h = currentPos.h;
        this.targetX = targetX;
        this.targetY = targetY;
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        isDriveWithinRange = (Math.abs(xError) <= DRIVE_POSITION_FUDGE) && (Math.abs(yError) <= DRIVE_POSITION_FUDGE);
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Action: ", "DRIVE");
        telemetry.addData("Pos X: ", this.x);
        telemetry.addData("Pos Y: ", this.y);
        telemetry.addData("Pox H: ", this.h);
        telemetry.addData("X Error: ", this.xError);
        telemetry.addData("Target X: ", this.targetX);
        telemetry.addData("Y Error: ", this.yError);
        telemetry.addData("Target Y: ", this.targetY);
        telemetry.addData("Drive Complete: ", this.isDriveWithinRange);
        telemetry.update();
    }
    
}
