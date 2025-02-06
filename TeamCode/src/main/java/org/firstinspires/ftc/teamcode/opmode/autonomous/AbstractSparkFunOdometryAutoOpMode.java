package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 *
 */
abstract class AbstractSparkFunOdometryAutoOpMode extends LinearOpMode {

    public double SPEED_GAIN  =  0.03;   // The default value ramps up to 50% power at a 25 inch error. (0.50 / 25.0)
    public double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double TURN_GAIN   =  0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)
    public double ROTATE_FUDGE_FACTOR = 2d; // Fudge factor for rotation - it will check to see if heading is within this amount

    private final ElapsedTime runtime = new ElapsedTime();

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

    public SparkFunOTOS.Pose2D myPosition() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        return new SparkFunOTOS.Pose2D(pos.x, pos.y, pos.h);
    }

    public void rotateRobot(double targetHeading, boolean shouldTurnClockwise) {
        double distanceToRotate = Math.abs(targetHeading - myPosition().h);
        double rotationPower = getRotationSpeed(distanceToRotate);
        robot.setDrivePower(shouldTurnClockwise ? rotationPower : -rotationPower,
                shouldTurnClockwise ? rotationPower : -rotationPower,
                shouldTurnClockwise ? -rotationPower : rotationPower,
                shouldTurnClockwise ? -rotationPower : rotationPower);
    }

    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        double drive, strafe;
        double xError, yError, yawError;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        boolean shouldRotate = (Math.abs(yawError) > ROTATE_FUDGE_FACTOR);
        boolean shouldTurnClockwise = MathUtils.shouldTurnClockwiseDegrees(myPosition().h, targetHeading);

        while(opModeIsActive() &&  (runtime.milliseconds() < maxTime*1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1) || shouldRotate )) {

            // Rotate to position
            if(shouldRotate) {
                telemetry.addData("Phase: ", "ROTATE");
                rotateRobot(targetHeading, shouldTurnClockwise);
            } else {
                telemetry.addData("Phase: ", "MOVE");
                double currentYawRadians = currentPos.h * 3.1415 / 180;
                double rotX = xError * Math.cos(currentYawRadians) - yError * Math.sin(currentYawRadians);
                double rotY = xError * Math.sin(currentYawRadians) + yError * Math.cos(currentYawRadians);

                drive = Range.clip(rotY * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(rotX * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                moveRobot(drive, strafe);
            }

            // then recalc error
            currentPos = myPosition();
            xError = targetX - currentPos.x;
            yError = targetY - currentPos.y;
            yawError = targetHeading - currentPos.h;

            // Did we just complete rotation? If so, stop motors immediately
            if(shouldRotate && (Math.abs(yawError) <= ROTATE_FUDGE_FACTOR)) {
                moveRobot(0, 0);
                shouldRotate = false;
            }

            telemetry.update();
        }
        moveRobot(0,0);
    }

    double getRotationSpeed(double distanceToRotate) {
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

    void moveRobot(double x, double y) {

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
