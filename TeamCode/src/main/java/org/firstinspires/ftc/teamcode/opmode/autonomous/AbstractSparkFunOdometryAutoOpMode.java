package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.autonomous.config.HeadingSource;
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

    private void rotateRobot(double targetHeading, boolean shouldTurnClockwise) {
        double distanceToRotate = Math.abs(targetHeading - myPosition().h);
        double rotationPower = getRotationSpeed(distanceToRotate);
        robot.setDrivePower(shouldTurnClockwise ? rotationPower : -rotationPower,
                shouldTurnClockwise ? rotationPower : -rotationPower,
                shouldTurnClockwise ? -rotationPower : rotationPower,
                shouldTurnClockwise ? -rotationPower : rotationPower);
    }

    public void autoDrive(double targetX, double targetY, int maxTime) {
        autoDrive(targetX, targetY, false, 0, maxTime);
    }

    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        autoDrive(targetX, targetY, true, targetHeading, maxTime);
    }

    public void autoDrive(double targetHeading, int maxTime) {
        SparkFunOTOS.Pose2D currentPos = myPosition();
        autoDrive(currentPos.x, currentPos.y, true, targetHeading, maxTime);
    }

    private boolean shouldDriveLoopContinue(int maxTime, boolean shouldDrive, boolean shouldRotate) {
        return (opModeIsActive() && runtime.milliseconds() < maxTime*1000 && (shouldDrive || shouldRotate));
    }

    private void logDriveMetrics(DriveError de) {
        telemetry.addData("Pos X: ", myPosition().x);
        telemetry.addData("Pos Y: ", myPosition().y);
        telemetry.addData("Pox H: ", myPosition().h);
        telemetry.addData("X Error: ", de.xError);
        telemetry.addData("Target X: ", de.targetX);
        telemetry.addData("Y Error: ", de.yError);
        telemetry.addData("Target Y: ", de.targetY);
        telemetry.addData("Yaw Error: ", de.yawError);
        telemetry.addData("Target Heading: ", de.targetHeading);
        telemetry.addData("Drive Complete: ", de.isDriveWithinRange);
        telemetry.addData("Rotate Complete: ", de.isHeadingWithinRange);
    }

    private void autoDrive(double targetX, double targetY, boolean adjustHeading, double targetHeading, int maxTime) {
        // Get Diff for all values
        DriveError driveError = new DriveError(myPosition(), targetX, targetY, targetHeading);

        runtime.reset();

        boolean shouldRotate = adjustHeading && !driveError.isHeadingWithinRange;
        boolean shouldDrive = !driveError.isDriveWithinRange;

        boolean shouldTurnClockwise = MathUtils.shouldTurnClockwiseDegrees(myPosition().h, targetHeading);
        while(shouldDriveLoopContinue(maxTime, shouldDrive, shouldRotate)) {

            // Rotate to position
            if(shouldRotate) {
                telemetry.addData("Phase: ", "ROTATE");
                rotateRobot(targetHeading, shouldTurnClockwise);
            } else {
                telemetry.addData("Phase: ", "MOVE");
                double drive  = Range.clip(driveError.yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double strafe = Range.clip(driveError.xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe);
            }

            // then recalculate drive error
            driveError = new DriveError(myPosition(), targetX, targetY, targetHeading);
            logDriveMetrics(driveError);

            // We just arrived at the correct location
            if(shouldDrive && driveError.isDriveWithinRange) {
                moveRobot(0, 0);
                shouldDrive = false;
                sleep(50);
            }

            // Did we just complete rotation? If so, stop motors immediately
            if(shouldRotate && driveError.isHeadingWithinRange) {
                moveRobot(0, 0);
                shouldRotate = false;
                sleep(50);
            }

            telemetry.update();
        }
        moveRobot(0,0);
        sleep(100);
    }

    private double getRotationSpeed(double distanceToRotate) {
        if(Math.abs(distanceToRotate) < 3) {
            return 0.1;
        }
        if(Math.abs(distanceToRotate) < 8) {
            return 0.25;
        }
        if(Math.abs(distanceToRotate) < 15) {
            return 0.5;
        }
        return 0.9;
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

class DriveError {

    public static double ROTATE_FUDGE_FACTOR = 2d;

    public static double DRIVE_POSITION_FUDGE = 1d;

    public double xError;

    public double yError;

    public double yawError;

    public double targetX;

    public double targetY;

    public double targetHeading;

    boolean isHeadingWithinRange;

    boolean isDriveWithinRange;

    DriveError(SparkFunOTOS.Pose2D currentPos, double targetX, double targetY, double targetHeading) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;
        isHeadingWithinRange = (Math.abs(yawError) <= ROTATE_FUDGE_FACTOR);
        isDriveWithinRange = (Math.abs(xError) <= DRIVE_POSITION_FUDGE) && (Math.abs(yError) <= DRIVE_POSITION_FUDGE);
    }
}
