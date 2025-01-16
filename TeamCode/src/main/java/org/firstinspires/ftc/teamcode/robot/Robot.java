package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends AbstractSparkFunRobot {

    // Motors --------------------------------------------------------

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor armMotor = null;
    public DcMotor vsMotor = null;

    // Servos --------------------------------------------------------

    public Servo wrist = null;
    public Servo claw = null;

    // Local variables -----------------------------------------------

    public int armPosition = RobotPosition.ARM_ORIGIN;
    private double armPower = 1.0d;
    public int vsPosition = 0;
    private double vsPower = 1.0d;
    public double clawPosition = RobotPosition.CLAW_CLOSED;
    public double wristPosition = RobotPosition.WRIST_IN;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

    @Override
    public void configureHardware() {
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

        // Will move the "attachments" to their starting positions
        setRobotAttachmentPositions();

        // Run configureHardware in parent class (which initializes sparkfun chip)
        super.configureHardware();
    }

    public void setArmPosition( int pos, double power ) {
        armPosition = pos;
        armPower = power;
        armMotor.setTargetPosition(armPosition);
        armMotor.setPower(armPower);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition( int pos ) {
        setArmPosition(pos, 1.0d);
    }

    public void setViperSlidePosition( int pos, double power ) {
        vsPosition = pos;
        vsPower = power;
        setViperSlidePositionWithFudgeFactor();
    }

    public void setViperSlidePosition( int pos ) {
        setViperSlidePosition(pos, 1.0d);
    }

    public void setClawPosition(double pos) {
        clawPosition = pos;
        claw.setPosition(clawPosition);
    }

    public void setWristPosition(double pos) {
        wristPosition = pos;
        wrist.setPosition(wristPosition);
    }

    public void setRobotAttachmentPositions() {
        // Arm motor
        armMotor.setTargetPosition(armPosition);
        armMotor.setPower(armPower);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Viper slide
        setViperSlidePositionWithFudgeFactor();

        // Claw
        claw.setPosition(clawPosition);

        // Wrist
        wrist.setPosition(wristPosition);
    }

    private void setViperSlidePositionWithFudgeFactor() {
        // Viper slide (with wiggle)
        // -----------------------------------------------------------
        // This will calculate the difference between the current position
        // and the target position. If the difference is greater than the
        // VIPER_SLIDE_WIGGLE value, then it will execute the command to go
        // to the target position.  This should avoid the jerky behavior
        // when trying to 'correct' the viper slide to the right position.
        int vsCurrentPosition = vsMotor.getCurrentPosition();
        int diff = Math.abs(vsPosition - vsCurrentPosition);
        int VIPER_SLIDE_WIGGLE = 40;
        if(diff > VIPER_SLIDE_WIGGLE) {
            vsMotor.setTargetPosition(vsPosition);
            vsMotor.setPower(vsPower);
            vsMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}
