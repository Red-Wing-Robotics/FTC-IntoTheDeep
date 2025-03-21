package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

public class PinpointOdometryProvider extends AbstractOdometryProvider {

    final private GoBildaPinpointDriver pinpoint;

    public PinpointOdometryProvider(String sensorName, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, sensorName);
    }

    @Override
    public void configure() {
        // Determine where to fit?
        pinpoint.setOffsets(0, 0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    @Override
    public Pose2D getPosition() {
        Pose2D pos = this.pinpoint.getPosition();
        return new Pose2D(distanceUnit, pos.getY(distanceUnit), pos.getX(distanceUnit), this.angleUnit, pos.getHeading(angleUnit));
    }

    @Override
    public void onLoop() {
        pinpoint.update();
    }

}
