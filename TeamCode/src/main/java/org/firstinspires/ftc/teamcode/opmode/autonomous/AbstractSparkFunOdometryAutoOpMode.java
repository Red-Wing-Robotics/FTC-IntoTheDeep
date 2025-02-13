package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.DriveState;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.HeadingSource;
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

    public double ROTATE_GAIN = 0.03; //

    public double MAX_AUTO_ROTATE = 0.8d;

    private final ElapsedTime runtime = new ElapsedTime();

    private HeadingSource headingSource = HeadingSource.SPARKFUN;

    public Robot robot = null;

    @Override
    public void runOpMode() {
        configureAutoDrive();

        robot = new Robot(hardwareMap, telemetry);
        robot.configureHardware();

        sleep(1000);

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
        autoDrive(targetX, targetY, false, 0, RotationDirection.CLOSEST, maxTime);
    }

    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        autoDrive(targetX, targetY, true, targetHeading, RotationDirection.CLOSEST, maxTime);
    }

    public void autoDrive(double targetHeading, int maxTime) {
        SparkFunOTOS.Pose2D currentPos = myPosition();
        autoDrive(currentPos.x, currentPos.y, true, targetHeading, RotationDirection.CLOSEST, maxTime);
    }

    public void autoDrive(double targetHeading, RotationDirection direction, int maxTime) {
        SparkFunOTOS.Pose2D currentPos = myPosition();
        autoDrive(currentPos.x, currentPos.y, true, targetHeading, direction, maxTime);
    }

    private boolean shouldDriveLoopContinue(int maxTime, boolean shouldDrive, boolean shouldRotate) {
        return (opModeIsActive() && runtime.milliseconds() < maxTime*1000 && (shouldDrive || shouldRotate));
    }

    private void logDriveMetrics(DriveState ds) {
        telemetry.addData("Pos X: ", myPosition().x);
        telemetry.addData("Pos Y: ", myPosition().y);
        telemetry.addData("Pox H: ", myPosition().h);
        telemetry.addData("X Error: ", ds.xError);
        telemetry.addData("Target X: ", ds.targetX);
        telemetry.addData("Y Error: ", ds.yError);
        telemetry.addData("Target Y: ", ds.targetY);
        telemetry.addData("Yaw Error: ", ds.yawError);
        telemetry.addData("Target Heading: ", ds.targetHeading);
        telemetry.addData("Drive Complete: ", ds.isDriveWithinRange);
        telemetry.addData("Rotate Complete: ", ds.isHeadingWithinRange);
    }

    private void autoDrive(double targetX, double targetY, boolean adjustHeading, double targetHeading, RotationDirection direction, int maxTime) {
        // Get Diff for all values
        DriveState driveState = new DriveState(myPosition(), targetX, targetY, targetHeading);

        runtime.reset();

        boolean shouldRotate = adjustHeading && !driveState.isHeadingWithinRange;
        boolean shouldDrive = !driveState.isDriveWithinRange;

        RotationDirection rotationDirection = (direction == RotationDirection.CLOSEST) ? MathUtils.getClosestRotationDirectionDegrees(myPosition().h, targetHeading) : direction;
        while(shouldDriveLoopContinue(maxTime, shouldDrive, shouldRotate)) {

            // Rotate to position
            if(shouldRotate) {
                telemetry.addData("Phase: ", "ROTATE");
                double distanceToRotate = Math.abs(targetHeading - myPosition().h);
                double power  = Range.clip(distanceToRotate * ROTATE_GAIN, -MAX_AUTO_ROTATE, MAX_AUTO_ROTATE);
                rotateRobot(power, rotationDirection);

            } else {
                telemetry.addData("Phase: ", "MOVE");
                double drive  = Range.clip(driveState.yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double strafe = Range.clip(driveState.xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                moveRobot(drive, strafe);
            }

            // then recalculate drive error
            driveState = new DriveState(myPosition(), targetX, targetY, targetHeading);
            logDriveMetrics(driveState);

            // We just arrived at the correct location
            if(shouldDrive && driveState.isDriveWithinRange) {
                moveRobot(0, 0);
                shouldDrive = false;
                sleep(50);
            }

            // Did we just complete rotation? If so, stop motors immediately
            if(shouldRotate && driveState.isHeadingWithinRange) {
                moveRobot(0, 0);
                shouldRotate = false;
                sleep(50);
            }

            telemetry.update();
        }
        moveRobot(0,0);
        sleep(100);
    }

    private void rotateRobot(double power, RotationDirection direction) {
        boolean isClockwise = (direction == RotationDirection.CLOCKWISE);
        robot.setDrivePower(isClockwise ? power : -power,
                isClockwise ? power : -power,
                isClockwise ? -power : power,
                isClockwise ? -power : power);
    }

    private void moveRobot(double x, double y) {

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
