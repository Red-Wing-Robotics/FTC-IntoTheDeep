package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Observation Zone Auto", group="RWR")
public class ObservationZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureRobot() {
        MAX_AUTO_SPEED = 0.8;
        MAX_AUTO_STRAFE = 0.8;
    }

    @Override
    void runOdometryDrive() {
        //otosDrive(5, 0, 0, 2);
        otosDrive(5, 22, 0, 2, (int)ARM_HANG_SPECIMEN, ARM_POWER, vsMotor.getCurrentPosition(), VIPER_SLIDE_POWER, claw.getPosition(), wrist.getPosition());
        drive(47, 22, 0, 2);
        drive(47, 30, 0, 2);
        drive(5, 30, 0, 2);
        drive(47, 30, 0, 2);
        drive(47, 39, 0, 2);
        drive(5, 39, 0, 2);
        //otosDrive(2, 10, 90, 2);
    }

}
