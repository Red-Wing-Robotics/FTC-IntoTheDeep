package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotPosition;

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
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        autoDrive(2,-7,0,2);
        autoDrive(2,0,0,2);
        autoDrive(49, 0, 0, 2);
        autoDrive(49, -11, 0, 2);
        autoDrive(5, -14, 0, 2);
        autoDrive(49, -11, 0, 2);
        autoDrive(49, -21, 0, 2);
        autoDrive(7, -21, 0, 2);
    }

}
