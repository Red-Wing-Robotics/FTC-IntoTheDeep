package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Observation Zone Auto", group="RWR")
public class ObservationZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureRobot() {
        SPEED_GAIN  =  0.1;
        STRAFE_GAIN =  0.2;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        //otosDrive(5, 0, 0, 2);
        drive(5, 22, 0, 2);
        drive(50, 22, 0, 2);
        drive(50, 30, 0, 2);
        drive(5, 30, 0, 2);
        drive(50, 30, 0, 2);
        drive(50, 39, 0, 2);
        drive(5, 39, 0, 2);
        drive(50, 39, 0, 2);
        drive(50, 48, 0, 2);
        drive(5, 48, 0, 2);
        //otosDrive(2, 10, 90, 2);
    }

}
