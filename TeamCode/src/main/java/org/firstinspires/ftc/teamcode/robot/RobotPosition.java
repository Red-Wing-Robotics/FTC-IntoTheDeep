package org.firstinspires.ftc.teamcode.robot;

public class RobotPosition {

    static public final double CLAW_CLOSED = 0.3d;
    static public final double CLAW_OPEN = 0.65d;
    static public final double WRIST_IN = 0.1d;
    static public final double WRIST_MID = 0.45d;
    static public final double WRIST_DOWN = 0.85d;
    static public final double ARM_TICKS_PER_DEGREE = 19.791666666667;
    static public final int ARM_ORIGIN = 0;
    static public final int ARM_SUBMERSIBLE = (int)(30 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_SUBMERSIBLE2 = (int)(40 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HANG_SPECIMEN = (int)(40 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HIGH_RUNG = (int)(75 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_LOW_BASKET = (int)(80 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HIGH_BASKET = (int)(110 * ARM_TICKS_PER_DEGREE);

}
