package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@Autonomous(name="Observation Zone Auto", group="RWR")
public class ObservationZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.1;
        STRAFE_GAIN =  0.2;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        autoDrive( 26, -14, 0, 2, RobotPosition.ARM_HIGH_RUNG, 1, 0, 1, RobotPosition.CLAW_CLOSED, RobotPosition.WRIST_MID);
        robot.setArmPosition( RobotPosition.ARM_HANG_SPECIMEN );
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        autoDrive(5, 22, 0, 2);
        autoDrive(50, 22, 0, 2);
        autoDrive(50, 30, 0, 2);
        autoDrive(5, 30, 0, 2);
        autoDrive(50, 30, 0, 2);
        autoDrive(50, 39, 0, 2);
        autoDrive(5, 39, 0, 2);
        autoDrive(50, 39, 0, 2);
        autoDrive(50, 48, 0, 2);
        autoDrive(5, 48, 0, 2);
    }

}
