package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.SleepUtils;

import java.util.Locale;

@SuppressWarnings("unused")
public class SparkFunOTOSOdometryProvider extends AbstractOdometryProvider {

    public final SparkFunOTOS.Pose2D startingPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    public final SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);

    private Pose2D position;

    private boolean updatePosition = true;


    protected SparkFunOTOS sparkFun;

    public SparkFunOTOSOdometryProvider(String sensorName, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.sparkFun = hardwareMap.get(SparkFunOTOS.class, sensorName);
    }

    public void configure() {
        // Setup OTOS
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        sparkFun.setLinearUnit(distanceUnit);
        sparkFun.setAngularUnit(angleUnit);
        sparkFun.setOffset(offset);
        double angularScalar = 1.00896539d;
        sparkFun.setAngularScalar(angularScalar);

        double linearScalar = 1.127d;
        sparkFun.setLinearScalar(linearScalar);
        sparkFun.calibrateImu();
        // Allow sleep for calibration to complete
        SleepUtils.sleep(2000);
        sparkFun.resetTracking();
        // Allow sleep to reset tracking (likely not needed)
        SleepUtils.sleep(2000);
        //sparkFun.setPosition(startingPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        sparkFun.getVersionInfo(hwVersion, fwVersion);

        // Log out OTOS information
        telemetry.addLine("OTOS configured!");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format(Locale.US, "OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    @Override
    public Pose2D getPosition() {
        // Won't fetch new position from sensor until the onLoop method is called, which
        // sets the updatePosition to true. This avoids unnecessary reads from the sensor,
        // and causes it to behave like the Pinpoint on position reads.
        if(updatePosition) {
            SparkFunOTOS.Pose2D pos = this.sparkFun.getPosition();
            return new Pose2D(this.distanceUnit, pos.x, pos.y, this.angleUnit, pos.h);
        }
        return position;
    }

    @Override
    public void onLoop() {
        updatePosition = true;
    }

}
