package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class AbstractRobot {

    protected final Telemetry telemetry;

    protected final HardwareMap hardwareMap;

    public AbstractRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        configureHardware();
    }

    protected abstract void configureHardware();

}
