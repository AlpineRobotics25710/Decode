package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static double FEED_TIME_SECONDS = 0.50; //The feeder servos run this long when a shot is requested.
    public static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_SPEED = 1.0;

    public static double INTAKE_POS = 0; // TODO: NEED TO GET VALUES
    public static double OUTTAKE_POS = 0.65; // TODO: NEED TO GET VALUES

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_TARGET_VELOCITY = 90;
    public static double LAUNCHER_MIN_VELOCITY = 80;
}
