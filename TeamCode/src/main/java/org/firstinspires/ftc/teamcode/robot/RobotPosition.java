package org.firstinspires.ftc.teamcode.robot;

public class RobotPosition {

    static public final double CLAW_CLOSED = 0.15d;
    static public final double CLAW_OPEN = 0.35d;
    static public final double CLAW_OPEN_AUTONOMOUS = 0.5;
    static public final double CLAW_OPEN_FOURTH_SAMPLE = 0.42;
    static public final double WRIST_IN = 0.85d;
    static public final double WRIST_SPECIMEN = 0.65d;
    static public final double WRIST_MID = 0.5d;
    static public final double WRIST_OFF_GROUND = 0.45;
    static public final double WRIST_DOWN = 0.25d;
    static public final double DEAD_WHEELS_DOWN = 0.2;
    static public final double DEAD_WHEELS_UP = 0.45;
    static public final double ARM_TICKS_PER_DEGREE = 19.791666666667;
    static public final int ARM_ORIGIN = 0;
    static public final int ARM_SPECIMEN = (int)(24 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_SUBMERSIBLE = (int)(32 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_SUBMERSIBLE2 = (int)(40 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HANG_SPECIMEN = (int)(61 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HANG_SPECIMEN_AUTO = (int)(60 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HIGH_RUNG = (int)(75 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_LOW_BASKET = (int)(80 * ARM_TICKS_PER_DEGREE);
    static public final int ARM_HIGH_BASKET = (int)(110 * ARM_TICKS_PER_DEGREE);
    static public final int VIPER_SLIDE_FULLY_EXTENDED = -2050;
    static public final int VIPER_SLIDE_EXPANSION_LIMIT = -1200;
    static public final int VIPER_SLIDE_ORIGIN = 0;
    static public final int VIPER_SLIDE_HANG_SPECIMEN = -500;
    static public final int VIPER_SLIDE_OFF_GROUND = -260;

}
