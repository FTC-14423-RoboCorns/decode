package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter.Subsystems.YawControl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret Yaw Control (YCTL)
 * Handles state management for aiming logic.
 */
@Config
public class YawControl {

    /** Internal turret state */
    public enum YawState {
        GENERAL_DIRECTION,
        APRILTAG
    }

    /** Alliance (to be moved into Robot or Competition class later) */
    public enum AimGoal {
        RED, BLUE
    }

    // === Hardware + Dependencies ===
    private final Servo yawServo;
    private final Telemetry telemetry;
    private final VisionSensor vision;   // External tracking/vision class // 

    // === Configurable constants (FTC Dashboard visible) ===
    public static double SERVO_INIT = 0.5;
    public static double SERVO_MIN = 0.1;
    public static double SERVO_MAX = 0.9;

    // === Aiming constants (replace with real coordinates later) ===
    public static double RED_GOAL_X = 72;
    public static double RED_GOAL_Y = -36;
    public static double BLUE_GOAL_X = -72;
    public static double BLUE_GOAL_Y = -36;

    // === Runtime state ===
    private YawState state = YawState.GENERAL_DIRECTION;
    public static AimGoal aimGoal = AimGoal.RED; // Default; can be set externally

    public YawControl(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;
        this.yawServo = hw.get(Servo.class, "yaw");
        this.vision = new VisionSensor(hw, telem);

        yawServo.scaleRange(SERVO_MIN, SERVO_MAX);
        yawServo.setPosition(SERVO_INIT);

        // aimGoal = Robot/Game.getAlliance() == Alliance.RED ?? AimGoal.RED : AimGoal.BLUE;
    }

    private void init(HardwareMap hardwareMap) {

        yawServo = hardwareMap.get(DcMotorEx.class, "YawServo");

        

    }

    // === Internal control ===
    private void telemetry() {
        telemetry.addData("YCTL Alliance", aimGoal);
        telemetry.addData("YCTL Aim State", state);
        telemetry.addData("YCTL Servo Pos", yawServo.getPosition());
    }

    private void aimGeneralDirectionUpdate() {
        // TODO: Abi’s general-direction aiming code
        // Should use robot’s (x, y, heading) and goal coordinates.
    }

    private void aimAprilTagUpdate() {
        // TODO: Abi’s AprilTag-based fine-aiming code
        // Example: uses vision.getTagYawOffset(), vision.getDistance(), etc.
    }

    private void resetAprilTagAimingParameters(){
        // I anticipate that there will be some PID constants to be reset
        // when switching from AprlitTag aiming to General Direction
        // and vice versa
    }

    private void resetGeneralDirectionAimingParameters(){
        // I anticipate that there will be some PID constants to be reset
        // when switching from AprlitTag aiming to General Direction
        // and vice versa
    }

    // === External control ===

    // Is used by controllers to set manually if needed

    public void setAlliance(AimGoal side) {
        aimGoal = side;
    }

    // === Main update loop ===
    public void update() {

        // Determine which aiming mode to use
        if (vision.detectedAprilTag()) {
            state = YawState.APRILTAG;
        } else {
            state = YawState.GENERAL_DIRECTION;
        }

        // Run correct aiming logic
        switch (state) {
            case GENERAL_DIRECTION:
                aimGeneralDirectionUpdate();
                break;

            case APRILTAG:
                aimAprilTagUpdate();
                break;
        }

        telemetry();
    }
}
