package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static double FEED_TIME_SECONDS = 0.50; //The feeder servos run this long when a shot is requested.
    public static double STOP_POWER = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_POWER = 1.0;

    public static double INTAKE_POWER = 0.65;

    public static double LAUNCHER_INTAKE_VELOCITY = -70;

    public static double RAMP_INTAKE_POS = 0.0; // TODO: NEED TO GET VALUES
    public static double RAMP_OUTTAKE_POS = 0.4; // TODO: NEED TO GET VALUES

    public static double BLOCKER_OPEN = 0.0; // TODO: NEED TO GET VALUES
    public static double BLOCKER_CLOSED = 0.36; // TODO: NEED TO GET VALUES

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_MAX_VELOCITY = 90;
    // public static double LAUNCHER_MIN_VELOCITY = 80; // Commented out because unnecessary (for now)
}

enum RampState {
    // Add more states if necessary
    INTAKE,
    OUTTAKE
}

enum BlockerState {
    OPEN,
    CLOSED
}
