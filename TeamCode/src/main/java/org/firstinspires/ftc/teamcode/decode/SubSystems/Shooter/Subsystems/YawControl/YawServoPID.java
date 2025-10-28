package org.firstinspires.ftc.teamcode.decode.SubSystems.Shooter.Subsystems.YawControl;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class YawServoPID
{
    private double Kp;
    private double Ki;
    private double Kd;
    double error = 0;
    private double out = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double derivative = 0;

    public YawServoPID(double Kp, double Ki, double Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
    public double getPos (double tx, ElapsedTime runTime, double turretPos)
    {
        double dt = runTime.seconds();
        runTime.reset();
        error = tx;

        if (Math.abs(error) < 0.2)
        {
            error = 0;
        }

        integralSum += error * dt;
        derivative = (error - lastError) / dt;
        lastError = error;

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        out *= dt; //slows it down to loop speed
        out = Range.clip(out, -0.0035, 0.0035);
        return Range.clip(turretPos + out, 0.0, 1.0);
    }
}