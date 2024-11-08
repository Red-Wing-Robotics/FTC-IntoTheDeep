package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arm Motor Test")
public class ArmMotorTest extends OpMode {
    DcMotor armMotor = null;
    final double ARM_TICKS_PER_DEGREE = 3.95849786281;
    final double TEST1 = 0 * ARM_TICKS_PER_DEGREE; // first test position
    final double TEST2 = 90 * ARM_TICKS_PER_DEGREE; // second test position
    final double[] ARM_POSITIONS = {TEST1, TEST2}; // test array to cycle through
    double armPos = 0;// creating and initializing the variable which the arm motor position will be set to
    boolean armForward = true;
    boolean armBackward = true;
    int armPosIdx = 0;
    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        /*
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

        armMotor.setTargetPosition((int)armPos);
        armMotor.setPower( 0.8 );
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        if(gamepad1.a) {
            armMotor.setTargetPosition((int) (0 * ARM_TICKS_PER_DEGREE));
            armMotor.setPower(0.3);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad1.b) {
            armMotor.setTargetPosition((int) (90 * ARM_TICKS_PER_DEGREE));
            armMotor.setPower(0.3);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        else if(gamepad1.x) {
            armMotor.setTargetPosition((int) (180 * ARM_TICKS_PER_DEGREE));
            armMotor.setPower(0.3);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
/* Put here in case it is needed
armMotor.setTargetPosition((int) ((180 * ARM_TICKS_PER_DEGREE) - armMotor.getPosition() ) );
 */