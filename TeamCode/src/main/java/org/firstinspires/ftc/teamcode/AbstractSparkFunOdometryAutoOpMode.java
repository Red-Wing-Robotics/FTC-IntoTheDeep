package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 *
 */
abstract class AbstractSparkFunOdometryAutoOpMode extends LinearOpMode {

    /**
     * The forward speed control. The default value ramps up to 50%
     * power at a 25 inch error. (0.50 / 25.0)
     */
    public double SPEED_GAIN  =  0.03;
    public double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double TURN_GAIN   =  0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    public SparkFunOTOS.Pose2D startingPosition = new SparkFunOTOS.Pose2D(0, 0, 0);

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);

    public DistanceUnit linearUnit = DistanceUnit.INCH;

    public AngleUnit angleUnit = AngleUnit.DEGREES;

    public double linearScalar = 1d;

    public double angularScalar = 1d;

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 drive motors.

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Sensors
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond

        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drive_leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drive_leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drive_rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drive_rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

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

    abstract void configureRobot();

    protected void configureOtos() {
        // Apply robot configurations
        configureRobot();

        // Setup OTOS
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();
        myOtos.setLinearUnit(linearUnit);
        myOtos.setAngularUnit(angleUnit);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(linearScalar);
        myOtos.setAngularScalar(angularScalar);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(startingPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        // Log out OTOS information
        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format(Locale.US, "OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }



    /**

     * Move robot to a designated X,Y position and heading

     * set the maxTime to have the driving logic timeout after a number of seconds.

     */

    void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {

        double drive, strafe, turn;

        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;

        double opp, adj;



        SparkFunOTOS.Pose2D currentPos = myPosition();

        xError = targetX-currentPos.x;

        yError = targetY-currentPos.y;

        yawError = targetHeading-currentPos.h;



        runtime.reset();



        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&

                ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) || (Math.abs(yawError) > 4)) ) {

            // Use the speed and turn "gains" to calculate how we want the robot to move.

            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;



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



            // Apply desired axes motions to the drivetrain.

            moveRobot(drive, strafe, turn);



            // then recalc error

            currentPos = myPosition();

            xError = targetX-currentPos.x;

            yError = targetY-currentPos.y;

            yawError = targetHeading-currentPos.h;

        }

        moveRobot(0,0,0);

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
        pos = myOtos.getPosition();
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.y, pos.x, -pos.h);
        return(myPos);
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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(10);
    }

}
