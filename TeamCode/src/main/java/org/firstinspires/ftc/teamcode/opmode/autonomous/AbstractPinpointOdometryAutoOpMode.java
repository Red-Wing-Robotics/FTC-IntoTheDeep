package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Robot;

abstract class AbstractPinpointOdometryAutoOpMode extends AbstractSparkFunOdometryAutoOpMode {

    @Override
    protected SparkFunOTOS.Pose2D myPosition() {
        Pose2D pos = robot.odo.getPosition();
        return new SparkFunOTOS.Pose2D(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
    }
    
}
