package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotPosition;
import org.firstinspires.ftc.teamcode.util.MovingAverageFilter;

/**
 * This class is designed to be used as our TeleOp OpMode when in a match. It has two differences
 * from the standalone version: it does not log position, and it does not completely reinitialize
 * the odometry provider (which takes time as we have to reset and configure the dead wheels and
 * the IMU.
 */
@SuppressWarnings("unused")
@TeleOp(name = "TeleOp Match", group = "RWR")
public class TeleOpMatch extends AbstractTeleOp {

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        // By setting this to false, we are not reinitializing the odometry provider
        // This is so that we can have a faster transition heading into the TeleOp
        // portion of the match.
        robot.configureHardware(false);
        df = new MovingAverageFilter(5);
        logPosition = false;
        robot.setDeadWheelRetractor(RobotPosition.DEAD_WHEELS_UP);
    }

}
