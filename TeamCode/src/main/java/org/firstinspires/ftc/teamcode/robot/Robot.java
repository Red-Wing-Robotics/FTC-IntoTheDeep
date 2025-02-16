package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends AbstractSparkFunRobot {

    // Motors --------------------------------------------------------

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor armMotor = null;
    public DcMotor vsMotor = null;

    // Servos --------------------------------------------------------

    public Servo wrist = null;
    public Servo claw = null;

    // Sensors -------------------------------------------------------

    public IMU imu = null; // BHI260AP

    // Local variables -----------------------------------------------

    public int armPosition = RobotPosition.ARM_ORIGIN;
    private double armPower = 1.0d;
    public int vsPosition = 0;
    private double vsPower = 1.0d;
    public double clawPosition = RobotPosition.CLAW_CLOSED;
    public double wristPosition = RobotPosition.WRIST_IN;

    private boolean isDriveEnabled = true;

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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vsMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //vsMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Will move the "attachments" to their starting positions
        setRobotAttachmentPositions();

        // Run configureHardware in parent class (which initializes sparkfun chip)
        super.configureHardware();
    }

    public void disableDriveControls() {
        setDrivePower(0,0,0,0);
        isDriveEnabled = false;
    }

    public void enableDriveControls() {
        isDriveEnabled = true;
    }

    public void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
        if(isDriveEnabled) {
            leftFrontDrive.setPower(frontLeft);
            leftBackDrive.setPower(backLeft);
            rightFrontDrive.setPower(frontRight);
            rightBackDrive.setPower(backRight);
        } else {
            telemetry.addData("DRIVE", "Cannot drive as drive disabled");
            telemetry.update();
        }
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
        vsMotor.setTargetPosition(vsPosition);
        vsMotor.setPower(vsPower);
        vsMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void scoreInHighBasket() {
        disableDriveControls();
        setWristPosition( RobotPosition.WRIST_MID );
        setViperSlidePosition( RobotPosition.VIPER_SLIDE_FULLY_EXTENDED );
        sleep(700);
        setWristPosition( RobotPosition.WRIST_DOWN );
        sleep(700);
        setClawPosition( RobotPosition.CLAW_OPEN );
        sleep(50);
        setWristPosition( RobotPosition.WRIST_SPECIMEN );
        sleep(500);
        setViperSlidePosition( 0 );
        enableDriveControls();
    }

    public void hangSpecimen(){
        disableDriveControls();
        setArmPosition( RobotPosition.ARM_HANG_SPECIMEN );
        setViperSlidePosition( RobotPosition.VIPER_SLIDE_HANG_SPECIMEN );
        setWristPosition( RobotPosition.WRIST_MID );
        setArmPosition( RobotPosition.ARM_HANG_SPECIMEN );
        sleep(850);
        enableDriveControls();
    }

}
