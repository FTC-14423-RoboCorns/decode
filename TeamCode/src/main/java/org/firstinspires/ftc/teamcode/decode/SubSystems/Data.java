package org.firstinspires.ftc.teamcode.decode.SubSystems;

import org.firstinspires.ftc.teamcode.util.Vector2D;

public class Data {

    //lastHeading, accessed by get and setHeading. Starts at 0.
    private static boolean red = true;
    private static Vector2D thePose = new Vector2D(0, 0);
    private static double heading;

    //Setter method
    public static void setRed(boolean redside) {
        red = redside;
    }

    //Getter method
    public static boolean getRed() {
        return red;
    }

    public static void setPose(Vector2D pose) {
        thePose.x = pose.x;
        thePose.y = pose.y;
    }

    public static Vector2D getPose() {
        return thePose;
    }

    public static void setHeading(double h) {
        heading = h;
    }

    public static double getHeading() {
        return heading;
    }
}
