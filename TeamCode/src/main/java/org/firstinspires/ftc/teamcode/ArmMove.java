package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Move")
public class ArmMove extends OpMode {
    DcMotor armMotor = null;
    DcMotor viperSlideMotor = null;
    double armPosition = 0;
    final double ARM_TICKS_PER_DEGREE = 19.791666666667;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "arm2");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_down)
        {
            armPosition += ARM_TICKS_PER_DEGREE;
        }
        else if(gamepad1.dpad_up)
        {
            armPosition -= ARM_TICKS_PER_DEGREE;
        }

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if( gamepad1.x)
            viperSlideMotor.setPower( 1 );
        else if ( gamepad1.y )
            viperSlideMotor.setPower( 1 );

        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition() / ARM_TICKS_PER_DEGREE);
        telemetry.update();

    }
}
