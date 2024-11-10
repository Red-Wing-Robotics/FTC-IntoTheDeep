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
    Servo wrist  = null;
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
    final double VIPER_SLIDE_POWER = 0.5d;
    //viper slide positions: low basket: 0; high basket: -1540; sumbersible: -1330 (approx); collect off ground: -302; high rung: 0
    final double ARM_TICKS_PER_DEGREE = 7.46805555555; // taken from GoBilda for test arm
    final double ORIGIN = 0 * ARM_TICKS_PER_DEGREE;

    final double SUBMERSIBLE = 20 * ARM_TICKS_PER_DEGREE;
    final double HIGH_RUNG = 75 * ARM_TICKS_PER_DEGREE;
    final double LOW_BASKET = 80 * ARM_TICKS_PER_DEGREE;
    final double HIGH_BASKET = 100 * ARM_TICKS_PER_DEGREE;
    final double[] ARM_POSITIONS = {ORIGIN, SUBMERSIBLE, HIGH_RUNG, LOW_BASKET, HIGH_BASKET}; // test array to cycle through
    double armPos = 0; // creating and initializing the variable which the arm motor position will be set to
    int armPosIdx = 0; // variable to track what index of ARM_POSITIONS is being used

    // variables telling whether or not the arm can be moved foreward or backward
    boolean armForward = true;
    boolean armBackward = true; 

    // TODO: CREATE CONSTANTS FOR POSITIONS FOR SERVOS
    final double CLAW_CLOSED = 0d;
    final double CLAW_OPEN = 0.3d;

    final double WRIST_IN = 0.5d; //Controlled by y
    final double WRIST_OUT = 0.85d; //controlled by x
    final double WRIST_MOVE = 0.1d;

    final double INTAKE_IN = -1d;
    final double INTAKE_OUT = -INTAKE_IN;

    @Override
    public void init() {
        // initializing servos
         wrist = hardwareMap.get(Servo .class, "wrist");
         claw = hardwareMap.get(Servo .class, "claw");
         activeIntake = hardwareMap.get(CRServo .class, "active intake");

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

         frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         // Setting up the test arm motor to RUN_TO_POSITION
         armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         // TODO: Set servos initial position
        wrist.setPosition(WRIST_IN);
        claw.setPosition(CLAW_CLOSED);
        activeIntake.setPower(0);
    }

    @Override
    public void loop() {

        
        /* commented out for testing
        if(gamepad1.dpad_up)
            armMotor.setPower(ARM_POWER);
        else if (gamepad1.dpad_down)
            armMotor.setPower(ARM_POWER * -1);
        else
            armMotor.setPower(0); */

        // test arm movements
        if(gamepad1.dpad_up && armForward && !gamepad1.dpad_down)
            {
                armForward = false;
                armBackward = true;
                armPosIdx = armPosIdx - 1; 
                
                // added to account for setting armPosIdx to -1
                if(armPosIdx < 0)
                    armPosIdx = ARM_POSITIONS.length - 1;
            }
        else if(gamepad1.dpad_down && armBackward && !gamepad1.dpad_up)
            {
                armForward = true;
                armBackward = false;
                armPosIdx = (armPosIdx + 1) % ARM_POSITIONS.length;
            }
        else if( !gamepad1.dpad_up && !gamepad1.dpad_down )
            {
                armBackward = true;
                armForward = true;
            }

        armPos = ARM_POSITIONS[ armPosIdx ];

        armMotor2.setTargetPosition((int)armPos);
        armMotor2.setPower(0.3);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData( "Arm Motor Position", armMotor2.getCurrentPosition() / ARM_TICKS_PER_DEGREE);

        if(gamepad1.x)
            viperSlideMotor.setPower(VIPER_SLIDE_POWER);
        else if (gamepad1.y)
            viperSlideMotor.setPower(VIPER_SLIDE_POWER * -1);
        else
            viperSlideMotor.setPower(0);

        telemetry.addData( "Viper Slide Motor Position", viperSlideMotor.getCurrentPosition());
        telemetry.update();

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

        // SERVOS
        // CLAW - Open A Close B
        // WRIST - Open X - Close Y (Increment by CONSTANT

        // TODO: Handle servo controls with gamepad
        if(gamepad2.dpad_down)
            activeIntake.setPower(INTAKE_IN);
        else if(gamepad2.dpad_up)
            activeIntake.setPower(INTAKE_OUT);
        else
            activeIntake.setPower(0);
            
        if(gamepad2.a)
            claw.setPosition(CLAW_OPEN);
        else if(gamepad2.b)
            claw.setPosition(CLAW_CLOSED);

        if(gamepad2.x)
            wrist.setPosition(WRIST_OUT);
        else if (gamepad2.y)
            wrist.setPosition(WRIST_IN);


    }

}
