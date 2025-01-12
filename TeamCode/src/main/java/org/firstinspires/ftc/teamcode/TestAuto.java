package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Autonomous", group="RWR")
public class TestAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureRobot() {
        MAX_AUTO_SPEED = 0.8;
        MAX_AUTO_STRAFE = 0.8;
    }

    @Override
    void runOdometryDrive() {
        setArmPosition( (int)ARM_HANG_SPECIMEN, ARM_POWER, 2 );
    }

}
