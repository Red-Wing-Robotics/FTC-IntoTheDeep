package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class DriveState {

    public final double xError;

    public final double yError;

    public final double yawError;

    public final double targetX;

    public final double targetY;

    public final double targetHeading;

    public final boolean isHeadingWithinRange;

    public final boolean isDriveWithinRange;

    public DriveState(SparkFunOTOS.Pose2D currentPos, double targetX, double targetY, double targetHeading) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;
        isHeadingWithinRange = (Math.abs(yawError) <= DriveConstants.ROTATE_FUDGE_FACTOR);
        isDriveWithinRange = (Math.abs(xError) <= DriveConstants.DRIVE_POSITION_FUDGE) && (Math.abs(yError) <= DriveConstants.DRIVE_POSITION_FUDGE);
    }
}
