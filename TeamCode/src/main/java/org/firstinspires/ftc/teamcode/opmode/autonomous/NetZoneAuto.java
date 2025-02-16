package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@Autonomous(name="Net Zone Auto", group="RWR")
public class NetZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.09;
        STRAFE_GAIN =  0.09;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        autoDrive(0,6, 0,2);
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-18,7,135,5);
        autoDrive(-19,6,135,1);
        robot.scoreInHighBasket();
        autoDrive(-11,18,90,2);
        sleep(200);
        autoDrive(-11,23,0,2);
        robot.setArmPosition(RobotPosition.ARM_ORIGIN);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setWristPosition( RobotPosition.WRIST_MID );
        sleep(400);
        autoDrive(-11,26,0,2);
        robot.setViperSlidePosition( -150 );
        sleep(400);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(400);
        robot.setViperSlidePosition( 0 );
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-20,6,135,5);
        autoDrive(-19,6,135,1);
        robot.scoreInHighBasket();

        autoDrive(-11,18,90,2);
        sleep(200);
        autoDrive(-22,23,0,2);
        robot.setArmPosition(RobotPosition.ARM_ORIGIN);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setWristPosition( RobotPosition.WRIST_MID );
        sleep(400);
        autoDrive(-22,25,0,2);
        robot.setViperSlidePosition( -150 );
        sleep(400);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(400);
        robot.setViperSlidePosition( 0 );
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-20,6,135,5);
        autoDrive(-19,6,135,1);
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
