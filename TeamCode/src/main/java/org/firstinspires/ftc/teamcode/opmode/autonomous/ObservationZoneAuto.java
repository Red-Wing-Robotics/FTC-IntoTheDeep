package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@Autonomous(name="Observation Zone Auto", group="RWR")
public class ObservationZoneAuto extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    void configureAutoDrive() {
        SPEED_GAIN  =  0.05;
        STRAFE_GAIN =  0.1;
        TURN_GAIN = 0.1;
        MAX_AUTO_SPEED = 1d;
        MAX_AUTO_STRAFE = 1d;
        MAX_AUTO_TURN = 1d;
    }

    @Override
    void runOdometryDrive() {
        robot.hangSpecimen();
        autoDrive( -5, 28, 0, 5);
        sleep(750);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setViperSlidePosition( 0 );
        sleep(700);
        autoDrive(-5, 18,0,5);
        robot.setArmPosition( RobotPosition.ARM_ORIGIN );
        robot.setWristPosition( RobotPosition.WRIST_IN );
        autoDrive(24, 18, 180, 5);
        autoDrive(24, 51, 180, 5);
        autoDrive(37, 51, 180, 5);
        autoDrive(37, 7, 180, 5);
        autoDrive(37, 51, 180, 5);
        autoDrive(44, 51, 180, 5);
        autoDrive(44, 7, 180, 5);
        autoDrive(44, 18, 180, 5);
        autoDrive(38, 18, 180, 5);
        robot.setArmPosition(RobotPosition.ARM_SUBMERSIBLE);
        robot.setClawPosition( RobotPosition.CLAW_OPEN );
        robot.setWristPosition( RobotPosition.WRIST_MID );
        autoDrive(38, 7, 180, 5);
        sleep(1000);
        robot.setClawPosition( RobotPosition.CLAW_CLOSED );
        sleep(500);
        robot.setArmPosition( RobotPosition.ARM_SUBMERSIBLE2 );
        robot.hangSpecimen();
        autoDrive(-7,8,0,5);
        autoDrive(-7,28,0,5);

    }

}
