package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotPosition;

@SuppressWarnings("unused")
@TeleOp(name = "Mecanum Viper Slide")
public class MecanumDriveViperSlide extends OpMode {

    //---------------------------------------------
    // CONSTANTS
    //---------------------------------------------

    // ARM ----------------------------------------

    final double[] ARM_POSITIONS = {
            RobotPosition.ARM_ORIGIN,
            RobotPosition.ARM_SUBMERSIBLE,
            RobotPosition.ARM_SUBMERSIBLE2,
            RobotPosition.ARM_HIGH_RUNG,
            RobotPosition.ARM_LOW_BASKET,
            RobotPosition.ARM_HIGH_BASKET
    }; // arm positions array to cycle through

    final double FUDGE_FACTOR = 5 * RobotPosition.ARM_TICKS_PER_DEGREE;

    //---------------------------------------------
    // LOCAL VARIABLES
    //---------------------------------------------

    // ARM ----------------------------------------

    double armPos = 0; // the variable which the arm motor position will be set to
    int posIdx = 0; // variable to track what index of ARM_POSITIONS is being used
    int vsPos = 0;

    // variables telling whether or not the arm can be moved forward or backward
    boolean armForward = true;
    boolean armBackward = true;

    double armPositionFudgeFactor = 0.0d;

    Robot robot;

    //---------------------------------------------
    // OPMODE OVERRIDDEN METHODS
    //---------------------------------------------

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
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

        robot.leftFrontDrive.setPower(frontLeftPower);
        robot.leftBackDrive.setPower(backLeftPower);
        robot.rightFrontDrive.setPower(frontRightPower);
        robot.rightBackDrive.setPower(backRightPower);
    }

    private void controlArm() {
        if (gamepad1.dpad_up && armForward && !gamepad1.dpad_down) {
            armForward = false;
            armBackward = true;
            posIdx = posIdx - 1;

            // added to account for setting armPosIdx to -1
            if (posIdx < 0) {
                posIdx = ARM_POSITIONS.length - 1;
            }
        } else if (gamepad1.dpad_down && armBackward && !gamepad1.dpad_up) {
            armForward = true;
            armBackward = false;
            posIdx = (posIdx + 1) % ARM_POSITIONS.length;
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            armBackward = true;
            armForward = true;
        }
        armPos = ARM_POSITIONS[posIdx];

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
        robot.setArmPosition((int) (armPos  + armPositionFudgeFactor));
        telemetry.addData("Arm Motor Position", robot.armMotor.getCurrentPosition() / RobotPosition.ARM_TICKS_PER_DEGREE);
    }

    private void controlViperSlide() {
        boolean canExtend = (robot.vsMotor.getCurrentPosition() > -1235 || robot.armMotor.getCurrentPosition() > 74 * RobotPosition.ARM_TICKS_PER_DEGREE);
        boolean canRetract = robot.vsMotor.getCurrentPosition() < 0;

        if (gamepad1.x && canRetract ) {
            vsPos++;
        } else if (gamepad1.y && canExtend ) {
            vsPos--;
        }

        // 'Fudge factor' or 'wiggle' handled in the Robot class
        // This should avoid the constant jerky nature of the viper slide
        if ( vsPos != robot.vsMotor.getCurrentPosition() ){
            robot.setViperSlidePosition(vsPos);
        }

        telemetry.addData("Viper Slide Motor Position", robot.vsMotor.getCurrentPosition());
    }

    private void controlServos() {
        if (gamepad2.a) {
            robot.setClawPosition(RobotPosition.CLAW_OPEN);
        } else if (gamepad2.b) {
            robot.setClawPosition(RobotPosition.CLAW_CLOSED);
        }

        if (gamepad2.x) {
            robot.setWristPosition(RobotPosition.WRIST_DOWN);
        } else if (gamepad2.y) {
            robot.setWristPosition(RobotPosition.WRIST_MID);
        }
    }

}
