package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel Control (FWCTL)
 * Handles spin-up, idle velocity holding, and shooting deceleration.
 */
@Config
public class FlywheelControl {

    public enum FWState {
        OFF,
        SPIN_UP, // Spin up to target speed
        READY, // The wheel spins at the target speed ± tolerance
        SHOOT // The turret shoots 1 ball; resets to SPIN_UP once the ball exits
    }

    private final DcMotorEx flywheel;
    private final Telemetry telemetry;

    private FWState state = FWState.OFF;

    // === Tunable constants (FTC Dashboard visible) ===
    public static double MAX_VELOCITY = -2500;   // ticks per second
    public static double TARGET_VELOCITY = -1800;
    public static double TOLERANCE = -150;   // Acceptable Δω around target velocity

    // === Ball exit detection ===

    private double lastVelocity = 0;

    // =================================================
    public FlywheelControl(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;
        this.flywheel = hw.get(DcMotorEx.class, "shooter");
        this.flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // === Internal Control ===

    private boolean isSpeedWithinTarget() {
        return Math.abs(flywheel.getVelocity() - TARGET_VELOCITY) <= Math.abs(TOLERANCE);
    }

    private boolean isShootRequested() {
        // Will be connected to some controller button
        return false;
    }

    private boolean isDoneShooting() {

        // Has to be done after each ball shooting to allow to
        // compensate for lost flywheel velocity

        // Detect that the ball has exited the turret by:
        // (1) measuring the speed drop in flywheel speed
        // (2) using the vision sensor
        // (3) using additional distance sensor at the exit
        // (4) using elapsed time
        // my suggestion is possibly using (1) in combination with (4),
        // since we don't know the time it takes to exit due to variable


        boolean startedIncreasing = true;
        lastVelocity = flywheel.getVelocity();

        return startedIncreasing;
    }

    private void telemetry(){
        telemetry.addData("FWCTL State", state);
        telemetry.addData("FWCTL Vel (tgt)", TARGET_VELOCITY);
        telemetry.addData("FWCTL Vel (cur)", flywheel.getVelocity());
    }

    // === External Control ===

    // Only should be used inside ShooterController to set it on or off.
    public void setState(FWState state) {
        this.state = state;
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public FWState getState(){
        return this.state;
    }

    // === Main update loop ===
    public void update() {

        double currentVelocity = flywheel.getVelocity();

        // Check if Off or On
        if (state == FWState.OFF) {
            flywheel.setVelocity(0);
            // return; // Possibly return early to save resources
        } else {
            flywheel.setVelocity(TARGET_VELOCITY);
        }

        // States bellow are never OFF

        // Set proper state
        if (state != FWState.SHOOT) {

            if (isSpeedWithinTarget()) {

                state = FWState.READY;

                if (isShootRequested()){ // add condition "&& !Storage/Intake.isEmpty()"
                    state = FWState.SHOOT;
                }

            } else {
                state = FWState.SPIN_UP;
            }

        } else {
            // state is SHOOT

            // IntakeController's Gate will detect it and open

            if (isDoneShooting()){
                state = FWState.SPIN_UP;
            }

        }

        telemetry();

    }
}
