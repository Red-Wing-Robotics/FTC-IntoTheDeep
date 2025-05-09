package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@Autonomous(name="Net Zone Auto", group="RWR")
public class NetZoneAuto extends AbstractAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.09;
        STRAFE_GAIN =  0.09;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        // First Sample
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-19,9,0,1);
        autoDrive(135,2);
        robot.scoreInHighBasket();
        // Second Sample
        autoDrive(-18,24,0,3);
        autoDrive(0,1);
        robot.setArmPosition(RobotPosition.ARM_ORIGIN);
        robot.setClawPosition( RobotPosition.CLAW_OPEN_AUTONOMOUS );
        robot.setWristPosition( RobotPosition.WRIST_OFF_GROUND );
        sleep(400);
        robot.setViperSlidePosition( 0 );
        sleep(700);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(700);
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-25,11,0,3);
        autoDrive(135,1);
        robot.scoreInHighBasket();
        // Third Sample
        autoDrive(-40,20,0,3);
        autoDrive(0,1);
        robot.setArmPosition(RobotPosition.ARM_ORIGIN);
        robot.setClawPosition( RobotPosition.CLAW_OPEN_AUTONOMOUS );
        robot.setWristPosition( RobotPosition.WRIST_OFF_GROUND );
        sleep(700);
        robot.setViperSlidePosition( 0 );
        sleep(400);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(900);
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-37,10,0,2);
        autoDrive(135,1);
        robot.scoreInHighBasket();
        // Fourth Sample
        sleep(400);
        autoDrive(-40,11, 0, 2);
        robot.setArmPosition(RobotPosition.ARM_SUBMERSIBLE);
        robot.setClawPosition( RobotPosition.CLAW_OPEN_FOURTH_SAMPLE );
        robot.setWristPosition( RobotPosition.WRIST_DOWN );
        autoDrive(30,1);
        sleep(300);
        robot.setViperSlidePosition( RobotPosition.VIPER_SLIDE_FULLY_EXTENDED );
        sleep(1000);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(700);
        robot.setViperSlidePosition( 0 );
        robot.setWristPosition( RobotPosition.WRIST_SPECIMEN );
        robot.setArmPosition( RobotPosition.ARM_HIGH_BASKET );
        autoDrive(-37,16,0,2);
        autoDrive(135,1);
        robot.scoreInHighBasket();
        autoDrive(0,1);

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
