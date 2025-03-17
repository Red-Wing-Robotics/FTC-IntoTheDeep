package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

abstract class AbstractOdometryProvider implements OdometryProvider {

    protected DistanceUnit distanceUnit = DistanceUnit.INCH;
    protected AngleUnit angleUnit = AngleUnit.DEGREES;

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    AbstractOdometryProvider(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    abstract public void configure();

    @Override
    public DistanceUnit getDistanceUnit() {
        return this.distanceUnit;
    }

    @Override
    public void setDistanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
    }

    @Override
    public AngleUnit getAngleUnit() {
        return this.angleUnit;
    }

    @Override
    public void setAngleUnit(AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
    }

    public void logPosition() {
        Pose2D pos = getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(this.linearUnit), pos.getY(this.linearUnit), pos.getHeading(this.angleUnit));
        telemetry.addData("Position", data);
    }

    abstract public Pose2D getPosition();

    abstract public void onLoop();

    protected final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
