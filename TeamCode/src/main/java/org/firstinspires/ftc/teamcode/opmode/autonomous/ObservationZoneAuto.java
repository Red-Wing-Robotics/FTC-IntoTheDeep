package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@Autonomous(name="Observation Zone Auto", group="RWR")
public class ObservationZoneAuto extends AbstractAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.09;
        STRAFE_GAIN =  0.09;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
    }

    @Override
    void runOdometryDrive() {
        // First Specimens
        robot.hangSpecimen();
        autoDrive( -5, 28, 2);
        sleep(100);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        sleep(500);
        // Push Samples
        autoDrive(-5, 18,0,4);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(35, 18, 0, 5);
        autoDrive(35, 36, 0, 2);
        autoDrive( 180, RotationDirection.COUNTER_CLOCKWISE, 1);
        autoDrive(58, -3, 180, 2);
        //autoDrive(48, 40, 180, 2);
        //autoDrive(62, 40, 180,2);
        //autoDrive(62, -1, 180, 2);
        //autoDrive(62, 20, 180, 2);
        autoDrive(52, 20, 180, 2);
        sleep(1000);
        // Second Specimen
        autoDrive(56, 4, 180, 2);
        robot.setWristPosition( RobotPosition.WRIST_MID );
        robot.setViperSlidePosition( 0 );
        robot.setClawPosition( RobotPosition.CLAW_OPEN_AUTONOMOUS );
        robot.setArmPosition( RobotPosition.ARM_SPECIMEN_AUTO );
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(400);
        robot.setArmPosition( RobotPosition.ARM_HANG_SPECIMEN_AUTO );
        robot.setViperSlidePosition( 0 );
        autoDrive(-11,10,0,5);
        robot.setArmPosition( RobotPosition.ARM_HANG_SPECIMEN );
        robot.hangSpecimen();
        autoDrive(-11,34, 0,2);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        autoDrive(-11,15, 0,5);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        // Third Specimen
        autoDrive(60, 20, 180, 5);
        robot.setViperSlidePosition( 0 );
        robot.setClawPosition( RobotPosition.CLAW_OPEN_AUTONOMOUS );
        robot.setArmPosition( RobotPosition.ARM_SPECIMEN_AUTO );
        sleep(1000);
        autoDrive(60,4, 180, 5);
        sleep(200);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(400);
        robot.setArmPosition( RobotPosition.ARM_HANG_SPECIMEN_AUTO );
        robot.setViperSlidePosition( 0 );
        autoDrive(-10, 9, 0, 5);
        robot.hangSpecimen();
        autoDrive(-10, 34, 0, 2);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        autoDrive(-17,24, 0,5);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        autoDrive(-60,1,180,3);
        /* Fourth Specimen
        autoDrive(50,25, 180,5);
        robot.setViperSlidePosition( RobotPosition.VIPER_SLIDE_OFF_GROUND );
        sleep(500);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(400);
        robot.setViperSlidePosition( 0 );
        autoDrive(-20, 15, 0, 2);
        robot.hangSpecimen();
        autoDrive(-20, 38, 0, 2);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 ); */
    }
}
