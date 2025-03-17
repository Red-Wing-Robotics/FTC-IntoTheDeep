package org.firstinspires.ftc.teamcode.util;

public class SleepUtils {

    static public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
