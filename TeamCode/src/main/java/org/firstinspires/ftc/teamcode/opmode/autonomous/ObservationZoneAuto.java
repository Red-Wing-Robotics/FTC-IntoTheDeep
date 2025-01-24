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
        autoDrive( 33, -14, 0, 2);
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        sleep(700);
        autoDrive(5, 22, 0, 2);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(50, 22, 0, 2);
        autoDrive(50, 22, 0, 2);
        autoDrive(50, 32, 0, 2);
        autoDrive(5, 32, 0, 2);
        autoDrive(50, 32, 0, 2);
        autoDrive(50, 41, 0, 2);
        autoDrive(5, 41, 0, 2);
        //autoDrive(55, 41, 0, 2);
        //autoDrive(55, 48, 0, 2);
        //autoDrive(5, 48, 0, 2);
    }

}
