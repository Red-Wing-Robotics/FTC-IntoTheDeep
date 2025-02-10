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
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        autoDrive(0,6,2);
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-16,6,135,5);
        robot.scoreInHighBasket();
        autoDrive(-12,16,2);
        robot.setArmPosition(RobotPosition.ARM_ORIGIN);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setWristPosition( RobotPosition.WRIST_MID );
        autoDrive( 0, 2);
        autoDrive(-12,27,2);
        robot.setViperSlidePosition( -150 );
        sleep(100);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(100);
        robot.setViperSlidePosition( 0 );
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-16,6,135,5);
        robot.scoreInHighBasket();
/*
        autoDrive(-7,2,0,2);
        autoDrive(0,2,0,2);
        autoDrive(0, 49, 0, 2);
        autoDrive(-11, 49, 0, 2);
        autoDrive(-14, 5, 0, 2);
        autoDrive(-11, 49, 0, 2);
        autoDrive(-21, 49, 0, 2);
        autoDrive(-21, 7, 0, 2); */
    }

}
