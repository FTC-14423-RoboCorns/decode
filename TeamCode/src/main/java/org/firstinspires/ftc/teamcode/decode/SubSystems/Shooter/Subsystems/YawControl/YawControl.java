package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter.Subsystems.YawControl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret Yaw Control (YCTL)
 * Handles state management for aiming logic.
 */
@Config
public class YawControl {

    /*
     * TODO:
     * 
     * Instead of there being multiple functions for each state,
     * Let it have one aiming function, aimTarget()
     * Let there be a private double target;
     * Then in update() depending on the state
     * It either changes the target to one of the presets
     * Or updates it relative to the general direction
     * 
     */

    /** Internal turret state */
    public enum YawState {
        GENERAL_DIRECTION,
        APRILTAG_AIMING,
        APRILTAG_LOCKED,
        MANUAL, // This one will be connected to buttons or joystics in case something goes wrong
        FIXED // This one will be used if something goes wring and we will just set it to always face front
    }

    /** Alliance (to be moved into Robot or Competition class later) */
    public enum AimGoal {
        RED, BLUE
    }

    // === Hardware + Dependencies ===
    private final Servo yawServo;
    private final Telemetry telemetry;
    private final Limelight3A limelight;   // External tracking/vision class //
    private ElapsedTime runtime = new ElapsedTime();
    // === Configurable constants (FTC Dashboard visible) ===
    public static double SERVO_INIT = 0.5;
    public static double SERVO_MIN = 0.1;
    public static double SERVO_MAX = 0.9;

    // === Aiming constants (replace with real coordinates later) ===
    public static double RED_GOAL_X = 72;
    public static double RED_GOAL_Y = -36;
    public static double BLUE_GOAL_X = -72;
    public static double BLUE_GOAL_Y = -36;
    public static double X_ERROR_TOLERANCE = 2;
    public double robotAngle = 0; // We should fetch it from the default state

    
    // === Runtime state ===
    private YawState state = YawState.GENERAL_DIRECTION;
    public static AimGoal aimGoal = AimGoal.RED; // Default; can be set externally
    private YawServoPID PID = new YawServoPID(0.013, 0.0000025, 0.0001); // Why is it its own class and not PIDFController?
    //TODO: Fix ServoPID with real world testing
    private double xError;
    public YawControl(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;
        this.yawServo = hw.get(Servo.class, "yaw");
        this.limelight = hw.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); //look every 100 ms
        limelight.start();
        limelight.pipelineSwitch(9);

        yawServo.scaleRange(SERVO_MIN, SERVO_MAX);
        yawServo.setPosition(SERVO_INIT);

    }

    private void init(HardwareMap hardwareMap) {

    }

    // === Internal control ===
    private void telemetry() {
        telemetry.addData("YCTL Alliance", aimGoal);
        telemetry.addData("YCTL Aim State", state);
        telemetry.addData("YCTL Servo Pos", yawServo.getPosition());
        telemetry.addData("YCTL Servo Degree", convertServoPos(yawServo.getPosition()));
    }

    private void aimGeneralDirectionUpdate() {
        double errorFromFront = robotAngle - 0;
        /* 0 needs to be substituted with the angle the robot has during the start
           this gives us the error from the front
         */
        //alr so this is just temporary bc we have no function to switch from red-blue yet
        if (aimGoal == AimGoal.RED)
            yawServo.setPosition(goToDegree( 45 - errorFromFront));
        if (aimGoal == AimGoal.BLUE)
            yawServo.setPosition(goToDegree( 90 - errorFromFront));
    }

    private void aimAprilTagUpdate() {
        //this should be everything except im unsure if runtime is properly being set
        //we wont know till testing :)
        yawServo.setPosition(PID.getPos(this.xError, runtime, yawServo.getPosition()));
    }

    public double convertServoPos(double servoPos)
    {
        //convert servo position to current degrees
        double y = (servoPos*-284.27208) + 231.88861;
        return y; //change
    }
    public double goToDegree(double targetDegree)
    {
        //convert degrees to servo position
        double servoPos = (targetDegree - 231.88861)/-284.27208;
        return servoPos;
    }

    //no reset

    // === External control ===

    // Is used by controllers to set manually if needed

    public void setAlliance(AimGoal side) {
        aimGoal = side;
    }

    // === Main update loop ===
    public void update(double orientation) {

        this.robotAngle = orientation;
        // Determine which aiming mode to use
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            state = YawState.APRILTAG_AIMING;
            this.xError = result.getTy();

            if(Math.abs(xError) < X_ERROR_TOLERANCE){
                state = YawState.APRILTAG_LOCKED;
            }
            
            //this is weird cus y error is actually x error since the cam is sideways
            //i might fix this later but if it works it works

        }
        else
        {
            state = YawState.GENERAL_DIRECTION;
        }

        // Run correct aiming logic
        switch (state) {
            case GENERAL_DIRECTION:
                aimGeneralDirectionUpdate();
                break;

            case APRILTAG_LOCKED:
            case APRILTAG_AIMING:
                aimAprilTagUpdate();
                break;

            case MANUAL:
                break;
            
            case FIXED:
                break;
        }


        telemetry();
    }
}
