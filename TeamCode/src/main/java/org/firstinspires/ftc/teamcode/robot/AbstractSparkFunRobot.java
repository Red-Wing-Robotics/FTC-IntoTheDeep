package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public abstract class AbstractSparkFunRobot extends AbstractRobot {

    public SparkFunOTOS myOtos;

    @SuppressWarnings("unused")
    private final SparkFunOTOS.Pose2D startingPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    private final SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    private final DistanceUnit linearUnit = DistanceUnit.INCH;
    private final AngleUnit angleUnit = AngleUnit.DEGREES;

    public AbstractSparkFunRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

    @Override
    public void configureHardware() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");
        configureOtos();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void configureOtos() {

        // Setup OTOS
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(linearUnit);
        myOtos.setAngularUnit(angleUnit);
        myOtos.setOffset(offset);
        double angularScalar = 1.00896539d;
        myOtos.setAngularScalar(angularScalar);

        double linearScalar = 1.127d;
        myOtos.setLinearScalar(linearScalar);
        myOtos.calibrateImu();
        // Allow sleep for calibration to complete
        sleep(2000);
        myOtos.resetTracking();
        // Allow sleep to reset tracking (likely not needed)
        sleep(2000);
        //myOtos.setPosition(startingPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        // Log out OTOS information
        telemetry.addLine("OTOS configured!");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format(Locale.US, "OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}
