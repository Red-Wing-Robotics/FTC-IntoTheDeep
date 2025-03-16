package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

abstract class AbstractPinpointOdometryAutoOpMode extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    protected SparkFunOTOS.Pose2D myPosition() {
        Pose2D pos = robot.odo.getPosition();
        return new SparkFunOTOS.Pose2D(pos.getY(DistanceUnit.INCH), pos.getX(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
    }

}
