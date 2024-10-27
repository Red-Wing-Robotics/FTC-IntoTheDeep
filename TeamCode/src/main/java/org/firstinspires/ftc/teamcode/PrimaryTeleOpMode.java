package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Primary TeleOp")
public class PrimaryTeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the motor
        DcMotor viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
        //the arm motor
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "left_arm");

        // Set the motor direction if necessary
        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Control the slide with the gamepad
            double viperSlidePower = gamepad1.left_stick_y; // Use the left stick to control the slide
            viperSlideMotor.setPower(viperSlidePower);

            double armPower = gamepad1.right_stick_y; // Use the left stick to control the slide
            viperSlideMotor.setPower(armPower);

            // Send telemetry data to the driver station
            telemetry.addData("Viper Slide Power", viperSlidePower);
            telemetry.addData("Arm Power", armPower);
            telemetry.update();
        }
    }

}
