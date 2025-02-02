package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;

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
    private void otosDrive(double targetX, double targetY, double targetHeading, int maxTime, int armPos, double armPower, int vsPos, double vsPower, double clawPos, double wristPos ) {

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
                turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

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
            robot.setArmPosition(armPos, armPower);
            robot.setViperSlidePosition(vsPos, vsPower);
            robot.setWristPosition(wristPos);
            robot.setClawPosition(clawPos);
            robot.setRobotAttachmentPositions();

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
        }

        // Stop robot movement
        moveRobot(0,0,0);

        // Set robot position
        currentPos = myPosition();

        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    SparkFunOTOS.Pose2D myPosition() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        //noinspection SuspiciousNameCombination
        return(new SparkFunOTOS.Pose2D(pos.y, pos.x, -pos.h));
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
        sleep(10);
    }

    public void autoDrive( double targetX, double targetY, double targetHeading, int maxTime) {
        otosDrive(
                targetX,
                targetY,
                targetHeading,
                maxTime,
                robot.armPosition,
                1.0d,
                robot.vsPosition,
                1.0d,
                robot.clawPosition,
                robot.wristPosition
        );
    }

    @SuppressWarnings("unused")
    public void autoDrive( double targetX, double targetY, double targetHeading, int maxTime, int armPos, double armPower, int vsPos, double vsPower, double clawPos, double wristPos ) {
        otosDrive(
                targetX,
                targetY,
                targetHeading,
                maxTime,
                armPos,
                armPower,
                vsPos,
                vsPower,
                clawPos,
                wristPos
        );
    }

}
