package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    final double CLAW_CLOSED = 0.3d;
    final double CLAW_OPEN = 0.6d;
    final double WRIST_IN = 0.1d;
    final double WRIST_MID = 0.45d;
    final double WRIST_DOWN = 0.85d;
    final double ARM_POWER = 1d;
    final double VIPER_SLIDE_POWER = 1d;
    final double ARM_TICKS_PER_DEGREE = 19.791666666667;
    final double ORIGIN = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_HANG_SPECIMEN = 40 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_RUNG = 75 * ARM_TICKS_PER_DEGREE;
    final double ARM_LOW_BASKET = 80 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_BASKET = 110 * ARM_TICKS_PER_DEGREE;

    public SparkFunOTOS.Pose2D startingPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    public DistanceUnit linearUnit = DistanceUnit.INCH;
    public AngleUnit angleUnit = AngleUnit.DEGREES;
    public double linearScalar = 1.121d;
    public double angularScalar = 1.018d;
    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 drive motors.

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotor armMotor = null;
    protected DcMotor vsMotor = null;
    protected Servo wrist = null;
    protected Servo claw = null;

    // Sensors
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        vsMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "arm2");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // set behavior flags for hardware
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vsMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction for drive
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vsMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vsMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");

        wrist.setPosition(WRIST_IN);
        claw.setPosition(CLAW_CLOSED);

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
        // Allow sleep for calibration to complete
        sleep(2000);
        myOtos.resetTracking();
        // Allow sleep to reset tracking (likely not needed)
        sleep(2000);
        myOtos.setPosition(startingPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        // Log out OTOS information
        telemetry.addLine("OTOS configured!");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format(Locale.US, "OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    void otosDrive(double targetX, double targetY, double targetHeading, int maxTime, int armPos, double armPower, int vsPos, double vsPower, double clawPos, double wristPos ) {

        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000)) {

            if ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) || (Math.abs(yawError) > 4)) {

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
            }

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
            telemetry.update(); d

            setArmPosition( armPos, armPower );
            setViperSlidePosition( vsPos, vsPower );
            claw.setPosition( clawPos );
            wrist.setPosition( wristPos );

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }

        //moveRobot(0,0,0);
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

    void setArmPosition( int pos, double power ) {
        armMotor.setTargetPosition(pos);
        armMotor.setPower(power);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setViperSlidePosition( int pos, double power ) {
        vsMotor.setTargetPosition(pos);
        vsMotor.setPower(power);
        vsMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void drive( double targetX, double targetY, double targetHeading, int maxTime){
        otosDrive( targetX, targetY, targetHeading, maxTime, armMotor.getCurrentPosition(), ARM_POWER, vsMotor.getCurrentPosition(), VIPER_SLIDE_POWER, claw.getPosition(), wrist.getPosition());
    }

}
