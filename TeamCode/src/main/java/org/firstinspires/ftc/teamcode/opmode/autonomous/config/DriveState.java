package org.firstinspires.ftc.teamcode.opmode.autonomous.config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometry.OdometryProvider;

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

    public DriveState(OdometryProvider odometryProvider, double targetX, double targetY) {
        Pose2D pos = odometryProvider.getPosition();
        x = pos.getX(odometryProvider.getDistanceUnit());
        y = pos.getY(odometryProvider.getDistanceUnit());
        h = pos.getHeading(odometryProvider.getAngleUnit());
        this.targetX = targetX;
        this.targetY = targetY;
        xError = targetX-this.x;
        yError = targetY-this.y;
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
