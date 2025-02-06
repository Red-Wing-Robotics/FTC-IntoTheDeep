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
        autoDrive(-7,2,0,2);
        autoDrive(0,2,0,2);
        autoDrive(0, 49, 0, 2);
        autoDrive(-11, 49, 0, 2);
        autoDrive(-14, 5, 0, 2);
        autoDrive(-11, 49, 0, 2);
        autoDrive(-21, 49, 0, 2);
        autoDrive(-21, 7, 0, 2);
    }

}
