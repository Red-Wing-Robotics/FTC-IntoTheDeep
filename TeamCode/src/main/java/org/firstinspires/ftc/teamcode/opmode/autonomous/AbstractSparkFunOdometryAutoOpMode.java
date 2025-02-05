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

    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */

    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        autoDrive(targetX, targetY, targetHeading, maxTime, "No Phase");
    }

    public void autoDrive(double targetX, double targetY, double targetHeading, int maxTime, String phase) {

        double drive, strafe, turn;
        double xError, yError, yawError;
        boolean hasArrived = false;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) && !hasArrived) {

            if ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) || (Math.abs(yawError) > 4)) {

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn = 0;//Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
            } else {
                hasArrived = true;
                moveRobot(0,0,0);
            }

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;

            // Set arm, claw, wrist, and slide positions
            robot.setRobotAttachmentPositions();

            // current x,y swapped due to 90 degree rotation
            telemetry.addData("Phase", phase);
            telemetry.addData("Status", "navigating");
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);

            telemetry.update();
        }

        // Stop robot movement
        moveRobot(0,0,0);

        // Set robot position
        currentPos = myPosition();

        telemetry.addData("Phase", phase);
        telemetry.addData("Status", "completed");
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.addData("target X coordinate", "");
        telemetry.addData("target Y coordinate", "");
        telemetry.addData("target Heading angle", "");
        telemetry.addData("xError", "");
        telemetry.addData("yError", "");
        telemetry.addData("yawError", "");
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
//    SparkFunOTOS.Pose2D myPosition() {
//        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
//        //noinspection SuspiciousNameCombination
//        return(new SparkFunOTOS.Pose2D(pos.y, pos.x, pos.h));
//        //return robot.myOtos.getPosition();
//    }

    public SparkFunOTOS.Pose2D myPosition() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        return new SparkFunOTOS.Pose2D(pos.x, pos.y, -pos.h);
    }

    final double ROTATE_FUDGE_FACTOR = 4.0d;

    public void rotateRobot(double targetHeading, int maxTime, String phase) {
        boolean hasArrived = false;
        runtime.reset();

        boolean shouldTurnClockwise = MathUtils.shouldTurnClockwise(Math.toRadians(myPosition().h), Math.toRadians(targetHeading));

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) && !hasArrived) {

            SparkFunOTOS.Pose2D currentPos = myPosition();
            double distanceToRotate = Math.abs(targetHeading - myPosition().h);

            if(!shouldKeepRotating(targetHeading)) {
                robot.setDrivePower(0,0,0,0);
                hasArrived = true;

                telemetry.addData("Phase", phase);
                telemetry.addData("Status", "completed");
                telemetry.addData("current X coordinate", currentPos.x);
                telemetry.addData("current X coordinate", currentPos.x);
                telemetry.addData("current Y coordinate", currentPos.y);
                telemetry.addData("current Heading angle", currentPos.h);
                telemetry.addData("target X coordinate", "");
                telemetry.addData("target Y coordinate", "");
                telemetry.addData("target Heading angle", targetHeading);
                telemetry.addData("xError", "");
                telemetry.addData("yError", "");
                telemetry.addData("yawError", distanceToRotate);
                telemetry.update();
            } else {
                double rotationPower = getRotationSpeed(distanceToRotate);
                robot.setDrivePower(shouldTurnClockwise ? rotationPower : -rotationPower,
                        shouldTurnClockwise ? rotationPower : -rotationPower,
                        shouldTurnClockwise ? -rotationPower : rotationPower,
                        shouldTurnClockwise ? -rotationPower : rotationPower);
            }

        }
        SparkFunOTOS.Pose2D currentPos = myPosition();

        telemetry.addData("Phase", phase);
        telemetry.addData("Status", "completed");
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.addData("target X coordinate", "");
        telemetry.addData("target Y coordinate", "");
        telemetry.addData("target Heading angle", "");
        telemetry.addData("xError", "");
        telemetry.addData("yError", "");
        telemetry.addData("yawError", "");
        telemetry.update();
    }

    public void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        //while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
        while(  (runtime.milliseconds() < maxTime*1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1) || (Math.abs(yawError) > 2)) ) {

            double currentYawRadians = currentPos.h*3.1415/180;
            double rotX = xError * Math.cos(currentYawRadians) - yError * Math.sin(currentYawRadians);
            double rotY = xError * Math.sin(currentYawRadians) + yError * Math.cos(currentYawRadians);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive  = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            strafe = Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            drive  = Range.clip(rotY * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(rotX * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
/*
            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();
*/
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        //currentPos = myPosition();
        /*
        telemetry.addData("current X c oordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
         */
    }

    double getRotationSpeed(double distanceToRotate) {
        if(Math.abs(distanceToRotate) < 5) {
            return 0.25;
        }
        if(Math.abs(distanceToRotate) < 10) {
            return 0.5;
        }
        return 1.0;
    }

    boolean shouldKeepRotating(double targetHeading) {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        return !(Math.abs(targetHeading - pos.h) < ROTATE_FUDGE_FACTOR);
    }

    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise: note this is not how the IMU reports yaw(heading)
     */
    void moveRobot(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

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
        //sleep(10);
    }

}
