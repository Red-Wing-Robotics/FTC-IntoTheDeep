package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("unused")
@TeleOp(name = "Mecanum Viper Slide")
public class MecanumDriveViperSlide extends OpMode {

    //---------------------------------------------
    // HARDWARE
    //---------------------------------------------

    // SERVOS -------------------------------------

    Servo wrist = null;
    Servo claw = null;

    // MOTORS -------------------------------------

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor viperSlideMotor = null;
    DcMotor armMotor2 = null;

    //---------------------------------------------
    // CONSTANTS
    //---------------------------------------------

    // ARM ----------------------------------------

    final double ARM_POWER = 1d;
    final double VIPER_SLIDE_POWER = 1d;
    final double ARM_TICKS_PER_DEGREE312 = 7.46805555555;
    final double ARM_TICKS_PER_DEGREE = 19.791666666667;
    final double ORIGIN = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_SUBMERSIBLE = 30 * ARM_TICKS_PER_DEGREE;
    final double ARM_SUBMERSIBLE2 = 40 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_RUNG = 75 * ARM_TICKS_PER_DEGREE;
    final double ARM_LOW_BASKET = 80 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_BASKET = 100 * ARM_TICKS_PER_DEGREE;
    final double[] ARM_POSITIONS = {ORIGIN, ARM_SUBMERSIBLE, ARM_SUBMERSIBLE2, ARM_HIGH_RUNG, ARM_LOW_BASKET, ARM_HIGH_BASKET}; // arm positions array to cycle through

    final double FUDGE_FACTOR = 5 * ARM_TICKS_PER_DEGREE;

    // CLAW ---------------------------------------

    final double CLAW_CLOSED = 0.3d;
    final double CLAW_OPEN = 1d;

    // WRIST --------------------------------------

    final double WRIST_IN = 0.1d; //Controlled by y
    final double WRIST_MID = 0.45d;
    final double WRIST_OUT = 0.85d; //controlled by x

    //---------------------------------------------
    // LOCAL VARIABLES
    //---------------------------------------------

    // ARM ----------------------------------------

    double armPos = 0; // the variable which the arm motor position will be set to
    int posIdx = 0; // variable to track what index of ARM_POSITIONS is being used

    // variables telling whether or not the arm can be moved forward or backward
    boolean armForward = true;
    boolean armBackward = true;

    double armPositionFudgeFactor = 0.0d;

    //---------------------------------------------
    // OPMODE OVERRIDDEN METHODS
    //---------------------------------------------

    @Override
    public void init() {
        // initializing servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // initializing motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
        armMotor2 = hardwareMap.get(DcMotor.class, "arm2"); // the 2 outlet in the expansion hub will be the test motor

        // set behavior flags for hardware
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction for drive
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setting up the test arm motor to RUN_TO_POSITION
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist.setPosition(WRIST_IN);
        claw.setPosition(CLAW_CLOSED);
    }

    @Override
    public void loop() {
        // Drive Functionality
        // GAMEPAD 1 : Left and Right Stick
        mecanumTankDrive();

        // Arm Functionality
        // GAMEPAD 1 : DPAD Up and Down, L,R Bottom Trigger
        controlArm();

        // Viper Slide Functionality
        // GAMEPAD 1 : X,Y
        controlViperSlide();

        // Servo Functionality
        // GAMEPAD 2 : X,Y,A,B
        controlServos();

        telemetry.update();
    }

    //---------------------------------------------
    // PRIVATE IMPLEMENTATION METHODS
    //---------------------------------------------

    private void setMotorPosition(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void mecanumTankDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.left_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void controlArm() {
        if (gamepad1.dpad_up && armForward && !gamepad1.dpad_down) {
            armForward = false;
            armBackward = true;
            posIdx = posIdx - 1;

            // added to account for setting armPosIdx to -1
            if (posIdx < 0)
                posIdx = ARM_POSITIONS.length - 1;

            // armPos = ARM_POSITIONS[ posIdx ];

            // setMotorPosition( armMotor2, (int)armPos, ARM_POWER );
            // setMotorPosition( viperSlideMotor, VS_POSITIONS[ posIdx ], VIPER_SLIDE_POWER);
        } else if (gamepad1.dpad_down && armBackward && !gamepad1.dpad_up) {
            armForward = true;
            armBackward = false;
            posIdx = (posIdx + 1) % ARM_POSITIONS.length;

            // armPos = ARM_POSITIONS[ posIdx ];

            // setMotorPosition( armMotor2, (int)armPos, ARM_POWER );
            // setMotorPosition( viperSlideMotor, VS_POSITIONS[ posIdx ], VIPER_SLIDE_POWER);
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            armBackward = true;
            armForward = true;
        }
        armPos = ARM_POSITIONS[posIdx];

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
        setMotorPosition(armMotor2, (int) (armPos  + armPositionFudgeFactor), ARM_POWER);
        telemetry.addData("Arm Motor Position", armMotor2.getCurrentPosition() / ARM_TICKS_PER_DEGREE);
    }

    private void controlViperSlide() {
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean canExtend = (viperSlideMotor.getCurrentPosition() > -1235 || armMotor2.getCurrentPosition() > 74);

        if (gamepad1.x) {
            // viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(VIPER_SLIDE_POWER);
        } else if (gamepad1.y && canExtend ) {
            // viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(VIPER_SLIDE_POWER * -1);
        } else {
            viperSlideMotor.setPower(0);
        }

        telemetry.addData("Viper Slide Motor Position", viperSlideMotor.getCurrentPosition());
    }

    private void controlServos() {
        if (gamepad2.a)
            claw.setPosition(CLAW_OPEN);
        else if (gamepad2.b)
            claw.setPosition(CLAW_CLOSED);

        if (gamepad2.x)
            wrist.setPosition(WRIST_OUT);
        else if (gamepad2.y)
            wrist.setPosition(WRIST_MID);
    }

}
