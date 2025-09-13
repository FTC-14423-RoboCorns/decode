 package org.firstinspires.ftc.teamcode.opmodes.testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.*;

@TeleOp()

public class TestSparkfunUltrasonic extends OpMode {
    public SparkfunUltrasonic rangeSensor;
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init( ) {
        rangeSensor = hardwareMap.get(SparkfunUltrasonic.class, "rear");
    }

    @Override
    public void start() {

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("raw ultrasonic", rangeSensor.readDistance());
        //telemetry.addData("raw optical", rangeSensor.rawOptical());
        //telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        //telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

    }

    @Override
    public void stop() {

    }
}