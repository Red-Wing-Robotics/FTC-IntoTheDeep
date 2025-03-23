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
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
         The most common tracking position is the center of the robot. <br> <br>
         The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
         the Y pod offset refers to how far forwards (in mm) from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
         @param xOffset how sideways from the center of the robot is the X (forward) pod? Left increases
         @param yOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
         */
        pinpoint.setOffsets(177,19.05);

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
