package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DpadTest")
public class DpadTest extends OpMode{
    @Override
    public void init() {}
    @Override
    public void loop() {
        
        if(gamepad1.dpad_up)
            telemetry.addData("dpad_up", true);
        else
            telemetry.addData("dpad_up", false);

        if(gamepad1.dpad_down)
            telemetry.addData("dpad_down", true);
        else
            telemetry.addData("dpad_down", false);

        telemetry.update();
    }
}
