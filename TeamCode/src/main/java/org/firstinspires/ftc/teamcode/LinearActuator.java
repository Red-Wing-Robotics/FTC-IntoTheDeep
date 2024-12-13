package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Linear Actuator")
public class LinearActuator extends OpMode {
    DcMotor linearA = null;

    @Override
    public void init() {
        linearA = hardwareMap.get( DcMotor.class, "linear actuator");
    }

    @Override
    public void loop() {
        if(gamepad1.x)
            linearA.setPower( 1d );
        else if (gamepad1.y)
            linearA.setPower( -1d );
        else
            linearA.setPower( 0d );
    }
}
