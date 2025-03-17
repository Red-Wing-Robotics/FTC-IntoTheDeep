package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.OdometryProvider;

public abstract class AbstractRobot {

    protected final Telemetry telemetry;

    protected final HardwareMap hardwareMap;

    public OdometryProvider odometryProvider;

    public AbstractRobot(HardwareMap hardwareMap, Telemetry telemetry, OdometryProvider odometryProvider) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.odometryProvider = odometryProvider;
    }

    public abstract void configureHardware();

}
