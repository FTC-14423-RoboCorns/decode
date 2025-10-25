package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ShooterController
 * Coordinates and manages all shooter-related subsystems:
 *  - Flywheel
 *  - Deflector
 *  - Feeder
 * 
 * Provides high-level control functions (aim, shoot, discard, stop, etc.)
 * and integrates data from submodules.
 */
@Config
public class ShooterController {

    // === Subsystems ===
    private final FlywheelControl flywheel;
    private final DeflectorControl deflector;
    private final YawControl yaw;

    // === Telemetry ===
    private final Telemetry telemetry;

    // === State variables ===
    private boolean isShooting = false;

    // === Constructor ===
    public ShooterController(Telemetry telem, DecodeRobot thisRobot) {
        this.telemetry = telem;
        this.flywheel = thisRobot.shooter.flywheel;
        this.deflector = thisRobot.shooter.deflector;
        this.feeder = thisRobot.shooter.feeder;
    }

    private void init(HardwareMap hardwareMap) {

        flywheel = new FlywheelControl();
        flywheel.init(hardwareMap);

        deflector = new DeflectorController(telem, this);
        deflector.init(hardwareMap);

        yaw = new YawController(telem, this);
        yaw.init(hardwareMap);

    }

    // === High-Level Control ===
    public void aimAtTarget(double robotX, double robotY, double targetX, double targetY) {
        // TODO: Integrate vision and geometry for aiming logic
    }

    public void startShooting() {
        // TODO: Start flywheel and feed sequence
    }

    public void stopShooting() {
        // TODO: Stop all shooter motion
    }

    public void discardShot() {
        // TODO: Lower deflector and shoot short-range
    }

    public void manualAdjust(double deflectorPos, double flywheelSpeed) {
        // TODO: Allow driver to manually control angle and speed
    }

    // === Update Loop ===
    public void update() {

        flywheel.update();
        deflector.update();
        yaw.update();
        
    }

    // === Telemetry ===
    public void telemetry() {
        // telemetry.addData("Shooter Active", isShooting);
        telemetry.update();
    }
}
