package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotPosition;
import org.firstinspires.ftc.teamcode.util.MovingAverageFilter;

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

    final double FUDGE_FACTOR = 2 * RobotPosition.ARM_TICKS_PER_DEGREE;

    //---------------------------------------------
    // LOCAL VARIABLES
    //---------------------------------------------

    // ARM ----------------------------------------

    double armPos = RobotPosition.ARM_ORIGIN; // the variable which the arm motor position will be set to
    int posIdx = 0; // variable to track what index of ARM_POSITIONS is being used
    int vsPos = RobotPosition.VIPER_SLIDE_ORIGIN;

    // variables telling whether or not the arm can be moved forward or backward
    boolean armForward = true;
    boolean armBackward = true;

    boolean scoringInBasket = false;
    boolean scoringOnChamber = false;

    double armPositionFudgeFactor = 0.0d;

    MovingAverageFilter df;

    Robot robot;

    //---------------------------------------------
    // OPMODE OVERRIDDEN METHODS
    //---------------------------------------------

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.configureHardware(false);
        df = new MovingAverageFilter(5);
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

        // Score in High Basket Method
        // GAMEPAD 2 Right Bumper
        scoreInHighBasket();

        // Set up for hanging specimen method
        // GAMEPAD 2 Left Bumper
        hangSpecimen();

        // Servo Functionality
        // GAMEPAD 2 : X,Y,A,B
        controlServos();

        // Log Position
        logPosition();

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
        robot.setDrivePower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    private void logPosition() {
        //df.add(robot.distanceSensor.getDistance(DistanceUnit.MM));
        //SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        //telemetry.addData("Pox X", pos.x);
        //telemetry.addData("Pos Y", pos.y);
        //telemetry.addData("Heading", pos.h);
        //telemetry.addData("Distance: ", df.getMovingAverage());
    }

    private void controlArm() {
        if( gamepad1.dpad_down ){
            armPos = RobotPosition.ARM_ORIGIN;
        } else if ( gamepad1.dpad_right ) {
            armPos = RobotPosition.ARM_SUBMERSIBLE;
        } else if ( gamepad1.dpad_up ) {
            armPos = RobotPosition.ARM_HANG_SPECIMEN;
        } else if ( gamepad1.dpad_left ) {
            armPos = RobotPosition.ARM_HIGH_BASKET;
        } else if ( gamepad1.x ) {
            armPos = RobotPosition.ARM_SPECIMEN;
        }

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
        robot.setArmPosition((int) (armPos  + armPositionFudgeFactor));
        telemetry.addData("Arm Motor Position", robot.armMotor.getCurrentPosition() / RobotPosition.ARM_TICKS_PER_DEGREE);
    }

    private void controlViperSlide() {

        telemetry.addData("Wrist Position: ", robot.wrist.getPosition());

        boolean isWristUp = robot.wrist.getPosition() > 0.3;
        boolean isViperSlideNotFullyExtended = robot.vsMotor.getCurrentPosition() > -1000;
        boolean isArmAboveLowestPosition = robot.armMotor.getCurrentPosition() > 74 * RobotPosition.ARM_TICKS_PER_DEGREE;
        boolean isViperSlideFullyExtended = robot.vsMotor.getCurrentPosition() > -1450;

        telemetry.addData("Is Wrist Down: ", isWristUp);
        telemetry.addData("Is VS Fully Extended: ", isViperSlideNotFullyExtended);
        telemetry.addData("Is Arm Above Lowest: ", isArmAboveLowestPosition);
        telemetry.addData("Is Viper Slide Fully Extended (Long Pos): ", isViperSlideFullyExtended);

        boolean canExtend = ((isViperSlideNotFullyExtended && isWristUp) ||
                isArmAboveLowestPosition ||
                (isViperSlideFullyExtended && !isWristUp));

        boolean canRetract = robot.vsMotor.getCurrentPosition() < 0;

        int VARIANCE = 50;

        if (gamepad2.dpad_down && canRetract ) {
            vsPos = Math.min(0, vsPos + VARIANCE);
        } else if (gamepad2.dpad_up && canExtend ) {
            vsPos = Math.max(RobotPosition.VIPER_SLIDE_FULLY_EXTENDED, vsPos - VARIANCE);
        }

        robot.setViperSlidePosition(vsPos);
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
            if( robot.vsMotor.getCurrentPosition() < RobotPosition.VIPER_SLIDE_EXPANSION_LIMIT && robot.armMotor.getCurrentPosition() <= 74 * RobotPosition.ARM_TICKS_PER_DEGREE)
            {
                vsPos = RobotPosition.VIPER_SLIDE_EXPANSION_LIMIT;
                robot.setViperSlidePosition( RobotPosition.VIPER_SLIDE_EXPANSION_LIMIT );
                sleep(200);
            }
            robot.setWristPosition(RobotPosition.WRIST_MID);
        }
    }

    private void scoreInHighBasket() {
        if( gamepad2.right_bumper && !scoringInBasket ) {
            scoringInBasket = true;
            robot.scoreInHighBasket();
            scoringInBasket = false;
        }
    }

    private void hangSpecimen(){
        if( gamepad2.left_bumper && !scoringOnChamber){
            scoringOnChamber = true;
            robot.hangSpecimen();
            vsPos = robot.vsMotor.getCurrentPosition();
            armPos = robot.armMotor.getCurrentPosition();
            scoringOnChamber = false;
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
