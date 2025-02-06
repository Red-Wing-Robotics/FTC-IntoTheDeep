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
        TURN_GAIN = 0.1;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
        MAX_AUTO_TURN = 1d;
    }

    @Override
    void runOdometryDrive() {
        robot.hangSpecimen();
        autoDrive( -14, 27, 0, 2);
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        sleep(700);
        autoDrive(-14, 20,0,2);
        autoDrive(22, 20, 0, 2);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(22, 50, 180, 2);
        autoDrive(32, 50, 180, 2);
        autoDrive(32, 5, 180, 2);
        autoDrive(32, 50, 180, 2);
        autoDrive(41, 50, 180, 2);
        autoDrive(41, 5, 180, 2);
        //autoDrive(55, 41, 0, 2);
        //autoDrive(55, 48, 0, 2);
        //autoDrive(5, 48, 0, 2);
    }

}
