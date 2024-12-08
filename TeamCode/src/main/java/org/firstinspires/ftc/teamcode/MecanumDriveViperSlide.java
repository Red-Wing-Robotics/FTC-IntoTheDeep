package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum Viper Slide")
public class MecanumDriveViperSlide extends OpMode {

    // Setup Servos
    //    Claw Expansion Hub 1 "claw"
    //    Wrist Expansion Hub 0 "wrist"
    Servo wrist = null;
    Servo claw = null;
    CRServo activeIntake = null;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    public DcMotor viperSlideMotor = null;
    public DcMotor armMotor = null;
    DcMotor armMotor2 = null;

    final double ARM_POWER = 1d;
    final double VIPER_SLIDE_POWER = 1d;

    final int VS_GROUND = -302;
    final int VS_SUBMERSIBLE = -1330;
    final int VS_HIGH_RUNG = 0;
    final int VS_LOW_BASKET = 0;
    final int VS_HIGH_BASKET = -1540;
    final int[] VS_POSITIONS = {VS_GROUND, VS_SUBMERSIBLE, VS_HIGH_RUNG, VS_LOW_BASKET, VS_HIGH_BASKET};
    final double ARM_TICKS_PER_DEGREE = 7.46805555555;
    final double ORIGIN = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_SUBMERSIBLE = 20 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_RUNG = 75 * ARM_TICKS_PER_DEGREE;
    final double ARM_LOW_BASKET = 80 * ARM_TICKS_PER_DEGREE;
    final double ARM_HIGH_BASKET = 100 * ARM_TICKS_PER_DEGREE;
    final double[] ARM_POSITIONS = {ORIGIN, ARM_SUBMERSIBLE, ARM_HIGH_RUNG, ARM_LOW_BASKET, ARM_HIGH_BASKET}; // arm positions array to cycle through
    double armPos = 0; // the variable which the arm motor position will be set to
    int posIdx = 0; // variable to track what index of ARM_POSITIONS is being used

    // variables telling whether or not the arm can be moved forward or backward
    boolean armForward = true;
    boolean armBackward = true;

    final double CLAW_CLOSED = 0d;
    final double CLAW_OPEN = 0.3d;

    final double WRIST_IN = 0.2d;
    final double WRIST_DOWN = 0.9d; //Controlled by y
    final double WRIST_OUT = 0.5d; //controlled by x
    final double WRIST_MOVE = 0.2d;

    final double INTAKE_IN = -1d;
    final double INTAKE_OUT = -INTAKE_IN;

    @Override
    public void init() {
        // initializing servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        activeIntake = hardwareMap.get(CRServo.class, "active intake");

        // initializing motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");
        armMotor2 = hardwareMap.get(DcMotor.class, "arm2"); // the 2 outlet in the expansion hub will be the test motor

        // set behavior flags for hardware
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setting up the test arm motor to RUN_TO_POSITION
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist.setPosition(WRIST_IN);
        claw.setPosition(CLAW_CLOSED);
        activeIntake.setPower(0);
    }

    public void setMotorPosition(DcMotor motor, int pos, double power) {
        motor.setTargetPosition(pos);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {

        // Drive Functionality
        mecanumTankDrive();

        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        setMotorPosition(armMotor2, (int) armPos, ARM_POWER);

        if (gamepad1.x) {
            // viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(VIPER_SLIDE_POWER);
        } else if (gamepad1.y) {
            // viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(VIPER_SLIDE_POWER * -1);
        } else
            viperSlideMotor.setPower(0);

        // SERVOS
        // ACTIVE INTAKE - In dpad_down - Out dpad_up
        // CLAW - Open A Close B
        // WRIST - Open X - Close Y

        if (gamepad2.dpad_down)
            activeIntake.setPower(INTAKE_IN);
        else if (gamepad2.dpad_up)
            activeIntake.setPower(INTAKE_OUT);
        else
            activeIntake.setPower(0);

        if (gamepad2.a)
            claw.setPosition(CLAW_OPEN);
        else if (gamepad2.b)
            claw.setPosition(CLAW_CLOSED);

        if (gamepad2.x)
            wrist.setPosition(WRIST_OUT);
        else if (gamepad2.y)
            wrist.setPosition(WRIST_DOWN);

        telemetry.addData("Arm Motor Position", armMotor2.getCurrentPosition() / ARM_TICKS_PER_DEGREE);
        telemetry.addData("Viper Slide Motor Position", viperSlideMotor.getCurrentPosition());
        telemetry.update();

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

}
