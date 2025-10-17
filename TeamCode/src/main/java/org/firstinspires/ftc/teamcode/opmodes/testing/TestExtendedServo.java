package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TestExtendedServo extends OpMode {

    public ServoImplEx armServo;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        armServo = hardwareMap.get(ServoImplEx.class, "rotator");
        armServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armServo.setPosition(1);
    }

    @Override
    public void start() {}

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        armServo.setPosition(0);
    }

    @Override
    public void stop() {}
}
