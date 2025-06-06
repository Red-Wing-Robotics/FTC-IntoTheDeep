package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.DriveState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.DriveToDistanceState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotateState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtils;
import org.firstinspires.ftc.teamcode.util.MovingAverageFilter;

abstract class AbstractAutoOpMode extends LinearOpMode {

    public double SPEED_GAIN  =  0.03;   // The default value ramps up to 50% power at a 25 inch error. (0.50 / 25.0)
    public double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_ROTATE = 1.0d;
    protected final ElapsedTime runtime = new ElapsedTime();

    public Robot robot = null;

    @Override
    public void runOpMode() {
        configureAutoDrive();

        robot = new Robot(hardwareMap, telemetry);
        robot.configureHardware();
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.vsMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(350);

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
        while(opModeIsActive()) {
            runOdometryDrive();
        }
    }

    abstract void runOdometryDrive();

    abstract void configureAutoDrive();

    public void autoDrive(double targetX, double targetY, int maxTimeSeconds) {
        driveRobot(targetX, targetY, maxTimeSeconds * 1000L);
    }

    // This does the extra work of splitting the max time between two separate calls,
    // one to driveRobot and one to rotateRobot
    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTimeSeconds) {
        long startingMillis = System.currentTimeMillis();
        rotateRobot(targetHeading, RotationDirection.CLOSEST, maxTimeSeconds * 1000L);
        long remainingMillis = (maxTimeSeconds * 1000L) - (System.currentTimeMillis() - startingMillis);
        if(remainingMillis < 0) {
            return;
        }
        driveRobot(targetX, targetY, remainingMillis);
    }

    public void autoDrive(double targetHeading, int maxTimeSeconds) {
        rotateRobot(targetHeading, RotationDirection.CLOSEST, maxTimeSeconds * 1000L);
    }

    @SuppressWarnings("unused")
    public void autoDrive(double targetHeading, RotationDirection direction, int maxTimeSeconds) {
        rotateRobot(targetHeading, direction, maxTimeSeconds * 1000L);
    }

    @SuppressWarnings("unused")
    public void autoDriveDistance(double targetHeading, double distanceMillimeters, int maxTimeSeconds) {
        autoDriveDistance(targetHeading, distanceMillimeters, 6, maxTimeSeconds);
    }

    public void autoDriveDistance(double targetHeading, double distanceMillimeters, int requiredSuccessiveReadings, int maxTimeSeconds) {
        long startingMillis = System.currentTimeMillis();
        rotateRobot(targetHeading, RotationDirection.CLOSEST, maxTimeSeconds * 1000L);
        long remainingMillis = (maxTimeSeconds * 1000L) - (System.currentTimeMillis() - startingMillis);
        if(remainingMillis < 0) {
            return;
        }
        driveToDistance(distanceMillimeters, requiredSuccessiveReadings, remainingMillis);
    }

    private void driveRobot(double targetX, double targetY, long maxTimeMilliseconds) {
        // Update position
        robot.odometryProvider.onLoop();

        // Get Diff for all values
        DriveState ds = new DriveState(robot.odometryProvider, targetX, targetY);

        double drive, strafe, currentYawRadians, rotX, rotY;

        runtime.reset();

        boolean shouldDrive = !ds.isDriveWithinRange;

        while(opModeIsActive() && runtime.milliseconds() < maxTimeMilliseconds && shouldDrive) {
            currentYawRadians = Math.toRadians(ds.h);
            rotY = ds.xError * Math.cos(currentYawRadians) - ds.yError * Math.sin(currentYawRadians);
            rotX = ds.xError * Math.sin(currentYawRadians) + ds.yError * Math.cos(currentYawRadians);

            // Drive is Y axis
            drive  = Range.clip(rotX * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            // Strafe is X axis
            strafe = Range.clip(rotY * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            setDrivePower(drive, -strafe);

            // Update position
            robot.odometryProvider.onLoop();

            // then recalculate drive error
            ds = new DriveState(robot.odometryProvider, targetX, targetY);
            ds.log(telemetry);

            // Did we just complete drive? If so, break from loop
            if(ds.isDriveWithinRange) {
                break;
            }
        }
        stopDriveMotors();
    }

    private void driveToDistance(double distanceToObject, int requiredSuccessiveReadings, long maxTimeMilliseconds) {
        // Update position
        robot.odometryProvider.onLoop();

        // Create moving average (low pass) filter
        MovingAverageFilter distanceFilter = new MovingAverageFilter(5);
        distanceFilter.add(robot.distanceSensor.getDistance(DistanceUnit.MM));

        // Get Diff for all values
        DriveToDistanceState dds = new DriveToDistanceState(distanceFilter.getMovingAverage(), distanceToObject);

        int successivePositiveReadings = 0;

        runtime.reset();

        boolean shouldDrive = !dds.isDistanceWithinRange;
        double power;

        while(opModeIsActive() && runtime.milliseconds() < maxTimeMilliseconds && shouldDrive) {

            if(!dds.isDistanceWithinRange) {
                if (Math.abs(dds.distanceError) < 100) {
                    power = 0.1;
                } else if (Math.abs(dds.distanceError) < 300) {
                    power = 0.25;
                } else {
                    power = 1;
                }
                setDrivePower((dds.distanceError > 0) ? power : -power, 0);
            }

            // then recalculate drive error
            distanceFilter.add(robot.distanceSensor.getDistance(DistanceUnit.MM));
            dds = new DriveToDistanceState(distanceFilter.getMovingAverage(), distanceToObject);
            dds.log(telemetry);

            // Update position
            robot.odometryProvider.onLoop();

            // Are we there? If so, start tracking successive readings.
            successivePositiveReadings = (dds.isDistanceWithinRange) ? successivePositiveReadings + 1 : 0;

            // Did we just hit the right number of successive readings? If so, break;
            if(successivePositiveReadings == requiredSuccessiveReadings) {
                break;
            }
        }
        stopDriveMotors();
    }

    private void rotateRobot(double targetHeading, RotationDirection direction, long maxTimeMilliseconds) {
        // Update position
        robot.odometryProvider.onLoop();

        // Get Diff for all values
        RotateState rs = new RotateState(robot.odometryProvider, targetHeading, direction);

        runtime.reset();

        boolean shouldRotate = !rs.isHeadingWithinRange;
        double power;

        if(direction == RotationDirection.CLOSEST) {
            direction = MathUtils.getClosestRotationDirectionDegrees(rs.h, targetHeading);
        }

        while(opModeIsActive() && runtime.milliseconds() < maxTimeMilliseconds && shouldRotate) {

            if(Math.abs(rs.yawError) < 10) {
                power = 0.2;
            } else if (Math.abs(rs.yawError) <25) {
                power = 0.5;
            } else {
                power = 1;
            }

            power = Math.min(power, MAX_AUTO_ROTATE);
            setRotatePower(power, direction);

            // Update position
            robot.odometryProvider.onLoop();

            // then recalculate drive error
            rs = new RotateState(robot.odometryProvider, targetHeading, direction);
            rs.log(telemetry);

            // Did we just complete rotation? If so, break from loop
            if(rs.isHeadingWithinRange) {
                break;
            }
        }
        stopDriveMotors();
    }

    private void stopDriveMotors() {
        robot.setDrivePower(0,0,0,0);
        // Force a stop to enable BRAKE
        sleep(10);
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
        double leftFrontPower = x - y;
        double rightFrontPower = x + y;
        double leftBackPower = x + y;
        double rightBackPower = x - y;

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
