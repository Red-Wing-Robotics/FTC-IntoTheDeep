package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@SuppressWarnings("unused")
@Autonomous(name="Net Zone Auto", group="RWR")
public class NetZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.1;
        STRAFE_GAIN =  0.2;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        autoDrive(49, 0, 0, 2);
        autoDrive(49, -9, 0, 2);
        autoDrive(0, -9, 0, 2);
        autoDrive(49, -9, 0, 2);
        autoDrive(49, -19, 0, 2);
        autoDrive(2, -19, 0, 2);
    }

}
