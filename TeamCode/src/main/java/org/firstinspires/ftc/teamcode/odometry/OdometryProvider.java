package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface OdometryProvider {

    DistanceUnit getDistanceUnit();

    void setDistanceUnit(DistanceUnit distanceUnit);

    AngleUnit getAngleUnit();

    void setAngleUnit(AngleUnit angleUnit);

    void logPosition();

    Pose2D getPosition();

    void configure();

    void onLoop();

}
