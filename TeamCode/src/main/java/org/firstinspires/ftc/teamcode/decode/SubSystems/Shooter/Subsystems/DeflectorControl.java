package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.PIDFController;

/**
 * Deflector Control (DCTL)
 * Manages the deflector (hood) angle for the turret.
 * Calculates target hood angle based on distance and flywheel velocity,
 * and manages manual and discard control modes.
 */
@Config
public class DeflectorControl {

    public enum AngleState {
        AIM,       // Calculate based on geometry and flywheel speed
        DISCARD,   // Fixed low-angle discard shot
        MANUAL     // Controlled manually by other inputs
    }

    private final Servo deflector;
    private final Telemetry telemetry;

    private AngleState state = AngleState.AIM;

    // === Tunable constants (FTC Dashboard) ===
    public static double SERVO_MIN = 0.1;
    public static double SERVO_MAX = 0.9;
    public static double SERVO_INIT = SERVO_MIN;
    public static double DISCARD_ANGLE = 0.2;      // position for discard shot
    
    // TODO: tune this contant and put it into DecodeRobot class or some new constants config class
    // It should be set to the one in the higher class, not to a number
    public static double SHOOTER_TO_GOAL_VERTICAL_DIST = 1; // meters
    // Same for this value
    public static double THRESHOLD_SHOOTER_SWITCH = 2;

    // === PIDF Controller ===
    // Is likely that we will not need it. Currently is not used anywhere
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.01;
    public static double kF = 0.0;

    private final PIDFController PIDF = new PIDFController(kP, kI, kD, kF);


    // === Runtime values ===
    private double targetAngle = SERVO_INIT;
    private double manualTarget = SERVO_INIT;

    // === External references ===
    private final FlywheelControl flywheel; // used to get velocity

    public DeflectorControl(HardwareMap hw, Telemetry telem, DecodeRobot thisRobot) {
        this.telemetry = telem;
        this.flywheel = thisRobot.shooter.flywheel;

        deflector.scaleRange(SERVO_MIN, SERVO_MAX);
        deflector.setPosition(SERVO_INIT);

    }

    private void init(HardwareMap hardwareMap) {

        deflector = hardwareMap.get(Servo.class, "Deflector");
        deflector.setPosition(DEFLECTOR_INIT);
        deflector.setDirection(Servo.Direction.FORWARD);

    }

    // === Internal Control ===
    private double computeTargetAngle() {

        double distance = 0; // Change to getting XY-plane distance from the vision sensor to the apriltag
        
        double tps = flywheel.getVelocity(); // ticks per second

        // Function: Ball Speed (m/s) = 0.003792 Motor Speed (TPS) + 0.425867
        double ball_speed = 0.03792 * tps + 0.425867

        double angle = calculateLaunchAngle(ball_speed, distance, SHOOTER_TO_GOAL_VERTICAL_DIST, THRESHOLD_SHOOTER_SWITCH);

        double servoPos = mapAngleToServoPos(angle);

        // Clamp result to servo range
        return clamp(SERVO_MIN, SERVO_MAX, servoPos);
    }

    private double calculateLaunchAngle(double v, double d, double dz, double threshold) {
        final double g = 9.81; // m/s²

        double term = Math.pow(v, 4) - g * (g * Math.pow(d, 2) + 2 * dz * Math.pow(v, 2));

        if (term < 0) return 0; // no valid shot, retract deflector

        double sqrtTerm = Math.sqrt(term);

        // Choose sign based on distance: use high arc for close shots
        double sign = (d < threshold) ? +1 : -1;

        double theta = Math.atan((Math.pow(v, 2) + sign * sqrtTerm) / (g * d));

        return Math.toDegrees(theta);
    }

    private double mapAngleToServoPos(double theta){
        //TODO: make a function that maps the angle of the deflector to the desired servo position
        return 0;
    }

    private double clamp (double min, double max, double val){
        return Math.max(min, Math.min(max, val));
    }


    // === External Control ===
    public void setState(AngleState newState) {
        this.state = newState;
    }

    public AngleState getState() {
        return state;
    }

    public void setManualTarget(double manualPos) {
        manualTarget = Math.max(SERVO_MIN, Math.min(SERVO_MAX, manualPos));
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    // === Main update loop ===
    public void update(double robotX, double robotY, double targetX, double targetY) {

        switch (state) {
            case AIM:
                targetAngle = computeTargetAngle();
                break;

            case DISCARD:
                targetAngle = DISCARD_ANGLE;
                break;

            case MANUAL:
                targetAngle = manualTarget;
                break;
        }

        deflector.setPosition(targetAngle);


        telemetry();
    }

    // === Telemetry ===
    private void telemetry() {
        telemetry.addData("DCTL State", state);
        telemetry.addData("DCTL Target Angle", targetAngle);
        telemetry.addData("DCTL Manual Target", manualTarget);
        telemetry.addData("DCTL Servo Pos", deflector.getPosition());
        telemetry.addData("DCTL Flywheel Vel", flywheel.getVelocity());
    }
}
