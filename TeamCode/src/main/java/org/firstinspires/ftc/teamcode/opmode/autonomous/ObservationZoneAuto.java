package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        robot.hangSpecimen();
        autoDrive( -5, 30, 2);
        sleep(100);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        sleep(500);
        autoDrive(-5, 18,0,4);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(24, 18, 180, 5);
        autoDrive(24, 51, 180, 2);
        autoDrive(33, 51, 180, 1);
        autoDrive(33, 8, 180, 2);
        //autoDrive(33, 52, 180, 2);
        //autoDrive(46, 52, 180, 1);
        //autoDrive(46, 8, 180,2);
        //autoDrive(46, 18, 180,2);
        autoDrive(38, 29, 180, 2);
        robot.setArmPosition(RobotPosition.ARM_SPECIMEN);
        autoDrive(38, 10, 180,2);
        //autoDriveDistance( 180, 280, 2);
        robot.setWristPosition( RobotPosition.WRIST_MID );
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(200);
        robot.setArmPosition( RobotPosition.ARM_SUBMERSIBLE2 );
        robot.hangSpecimen();
        autoDrive(-7,10,0,5);
        autoDrive(-7,31, 0,2);

        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(-7,10, 0,5);
        //robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        autoDrive(38, 18, 180, 5);
        robot.setArmPosition(RobotPosition.ARM_SPECIMEN);
        //robot.setClawPosition( RobotPosition.CLAW_OPEN );
        //robot.setWristPosition( RobotPosition.WRIST_MID );
        autoDrive(38, 10, 180, 2);
        //autoDrive( -175, 1);
        //autoDriveDistance( -175, 280, 2);
        robot.setWristPosition( RobotPosition.WRIST_MID );
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(200);
        robot.setArmPosition( RobotPosition.ARM_SUBMERSIBLE2 );
        robot.hangSpecimen();
        autoDrive(-7,10,0,5);
        autoDrive(-7,31,2);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        autoDrive(-7,10,0,5);
        autoDrive(38, 5, 0, 2);
    }

}
