package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MovingAverageFilter;

/**
 * This class is designed to be used whenever we need a standalone TeleOp OpMode. This has two
 * distinct differences from the one we use for matches: it logs out the position, and it allows for
 * a full configuration of the odometry provider.
 */
@SuppressWarnings("unused")
@TeleOp(name = "TeleOp Standalone", group = "RWR")
public class TeleOpStandalone extends AbstractTeleOp {

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.configureHardware(true);
        df = new MovingAverageFilter(5);
        logPosition = true;
    }

}
