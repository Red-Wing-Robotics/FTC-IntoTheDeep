package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractSparkFunOdometryAutoOpMode;

@Autonomous(name="Net Zone Auto", group="RWR")
public class NetZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureRobot() {
        MAX_AUTO_SPEED = 0.8;
        MAX_AUTO_STRAFE = 0.8;
    }

    @Override
    void runOdometryDrive() {
        //otosDrive(5, 0, 0, 2);
        otosDrive(49, 0, 0, 2);
        otosDrive(49, -9, 0, 2);
        otosDrive(0, -9, 0, 2);
        otosDrive(49, -9, 0, 2);
        otosDrive(49, -19, 0, 2);
        otosDrive(2, -19, 0, 2);
        //otosDrive(2, 10, 90, 2);
    }

}
