package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.DriveState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.HeadingSource;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotateState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 *
 */
abstract class AbstractSparkFunOdometryAutoOpMode extends LinearOpMode {

    public double SPEED_GAIN  =  0.03;   // The default value ramps up to 50% power at a 25 inch error. (0.50 / 25.0)
    public double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)

    public double ROTATE_GAIN = 0.01; //

    public double MAX_AUTO_ROTATE = 0.6d;

    private final ElapsedTime runtime = new ElapsedTime();

    private HeadingSource headingSource = HeadingSource.SPARKFUN;

    public Robot robot = null;

    @Override
    public void runOpMode() {
        configureAutoDrive();

        robot = new Robot(hardwareMap, telemetry);
        robot.configureHardware();

        sleep(500);

        while(!isStarted()) {
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Execute the Autonomous Commands
        runOdometryDrive();
    }

    abstract void runOdometryDrive();

    abstract void configureAutoDrive();

    public void setHeadingSource(HeadingSource source) {
        headingSource = source;
    }

    // This is an expensive call - we should only do this once per loop
    public SparkFunOTOS.Pose2D myPosition() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        if(headingSource == HeadingSource.SPARKFUN) {
            return new SparkFunOTOS.Pose2D(pos.x, pos.y, pos.h);
        } else {
            double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            return new SparkFunOTOS.Pose2D(pos.x, pos.y, heading);
        }
    }

    public void autoDrive(double targetX, double targetY, int maxTime) {
        driveRobot(targetX, targetY, maxTime);
    }

    // Note that max time for this call will be used for both rotate and drive
    // It doesn't keep a max time for both actions together, but rather for each
    // of them individually.
    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        rotateRobot(targetHeading, RotationDirection.CLOSEST, maxTime);
        driveRobot(targetX, targetY, maxTime);
    }

    public void autoDrive(double targetHeading, int maxTime) {
        rotateRobot(targetHeading, RotationDirection.CLOSEST, maxTime);
    }

    public void autoDrive(double targetHeading, RotationDirection direction, int maxTime) {
        rotateRobot(targetHeading, direction, maxTime);
    }

    private boolean shouldLoopContinue(int maxTime, boolean shouldContinue) {
        return (opModeIsActive() && runtime.milliseconds() < maxTime*1000 && shouldContinue);
    }

    private void driveRobot(double targetX, double targetY, int maxTime) {
        // Get Diff for all values
        DriveState ds = new DriveState(myPosition(), targetX, targetY);

        runtime.reset();

        boolean shouldContinue = !ds.isDriveWithinRange;

        while(shouldLoopContinue(maxTime, shouldContinue)) {

            double currentYawRadians = Math.toRadians(ds.h);
            double rotY = ds.xError * Math.cos(currentYawRadians) - ds.yError * Math.sin(currentYawRadians);
            double rotX = ds.xError * Math.sin(currentYawRadians) + ds.yError * Math.cos(currentYawRadians);
            setDrivePower(rotX, rotY);

            // We just arrived at the correct location
            if(ds.isDriveWithinRange) {
                setDrivePower(0, 0);
                shouldContinue = false;
                sleep(50);
            }

            // then recalculate drive error
            ds = new DriveState(myPosition(), targetX, targetY);
            ds.log(telemetry);
        }

        setDrivePower(0,0);
        sleep(50);
    }

    private void rotateRobot(double targetHeading, RotationDirection direction, int maxTime) {
        // Get Diff for all values
        RotateState rs = new RotateState(myPosition(), targetHeading);

        runtime.reset();

        boolean shouldContinue = !rs.isHeadingWithinRange;

        if(direction == RotationDirection.CLOSEST) {
            direction = MathUtils.getClosestRotationDirectionDegrees(rs.h, targetHeading);
        }

        while(shouldLoopContinue(maxTime, shouldContinue)) {
            double distanceToRotate = Math.abs(targetHeading - rs.h);
            double power  = Range.clip(distanceToRotate * ROTATE_GAIN, -MAX_AUTO_ROTATE, MAX_AUTO_ROTATE);
            setRotatePower(power, direction);

            // Did we just complete rotation? If so, stop motors immediately
            if(rs.isHeadingWithinRange) {
                setRotatePower(0, RotationDirection.CLOCKWISE);
                shouldContinue = false;
                sleep(50);
            }

            // then recalculate drive error
            rs = new RotateState(myPosition(), targetHeading);
            rs.log(telemetry);
        }

        setRotatePower(0, RotationDirection.CLOCKWISE);
        sleep(50);
    }

    private void setRotatePower(double power, RotationDirection direction) {
        boolean isClockwise = (direction == RotationDirection.CLOCKWISE);
        robot.setDrivePower(isClockwise ? power : -power,
                isClockwise ? power : -power,
                isClockwise ? -power : power,
                isClockwise ? -power : power);
    }

    private void setDrivePower(double x, double y) {

        // Calculate wheel powers.
        double leftFrontPower    =  x +y;
        double rightFrontPower   =  x -y;
        double leftBackPower     =  x -y;
        double rightBackPower    =  x +y;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

}
