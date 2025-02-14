package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RotateState {

    public final double x;

    public final double y;

    public final double h;

    public final double yawError;

    public final double targetHeading;

    public final boolean isHeadingWithinRange;

    public RotateState(SparkFunOTOS.Pose2D currentPos, double targetHeading) {
        x = currentPos.x;
        y = currentPos.y;
        h = currentPos.h;
        this.targetHeading = targetHeading;
        yawError = targetHeading-currentPos.h;
        isHeadingWithinRange = (Math.abs(yawError) <= DriveConstants.ROTATE_FUDGE_FACTOR);
    }
    
    public void log(Telemetry telemetry) {
        telemetry.addData("Action: ", "ROTATE");
        telemetry.addData("Pos X: ", this.x);
        telemetry.addData("Pos Y: ", this.y);
        telemetry.addData("Pox H: ", this.h);
        telemetry.addData("Yaw Error: ", this.yawError);
        telemetry.addData("Target Heading: ", this.targetHeading);
        telemetry.addData("Rotate Complete: ", this.isHeadingWithinRange);
        telemetry.update();
    }

}
