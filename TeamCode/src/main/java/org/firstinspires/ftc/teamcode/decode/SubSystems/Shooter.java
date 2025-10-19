package org.firstinspires.ftc.teamcode.decode.SubSystems;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Shooter {
    public DcMotorEx shooter;
    public Servo deflector;
    public Telemetry telemetry;
    //TODO: confirm mag
    private boolean debug = false;
    public double angle;

    public double liftPos;
    public boolean shooting;

    //the location of the red goal on the plane
    public final static double goalX = 72;
    public final static double goalY = -36;

    public boolean isShooterOn = false;
    public boolean isDoneShooting;

    private final int FAR_VELOCITY = -1800;
    private final int NEAR_VELOCITY = -1600;//2000;
    private final int MAX_VELOCITY = -2500;
    private final int NEAR=48;//need to set distance for near shooting
    public final double DEFLECTOR_INIT = 0;

    //the location of the middle of the robot.
    public static double robotY = 0;
    public static double robotX = 0;

    //Is the robot on the red side(true) or blue side(false)
    private int isBlue = 1; // Why is it not a boolean?

    //height of the goal

    public Shooter(HardwareMap hardwareMap, Telemetry telem) {
        init(hardwareMap);
        //will want to instantiate the turret object, perhaps by passing robot in to the shooter constructor.
        //much of our shooter code will depend on querying turret
        //will likely need robot here as well because our "pusher" is the intake
        isShooterOn = false;

        this.telemetry = telem;
        if (Data.getBlue()) {

            isBlue = 1;

        } else {

            isBlue = -1;
        }
    }

    private void init(HardwareMap hardwareMap) {
        //TODO: confirm mag class
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        deflector = hardwareMap.get(Servo.class, "deflector");
        deflector.setPosition(DEFLECTOR_INIT);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }


    private double deflectorHeight(double distancetogoal) {
        double tempHeight
        //here is where we will put our calculations to convert our distance to goal to a servo position for our deflector
        return tempHeight;
    }


    //three versions of this method
    //no parameter: go to default (FAR) velocity
    //int parameter: sets velocity to the parameter
    //double parameter between 0 and 1: sets velocity to percent of max
    //returns the velocity set, as caller may not know default or max.
    public double shooterOn() {
        ((DcMotorEx) shooter).setVelocity(FAR_VELOCITY);
        isShooterOn = true;
        return FAR_VELOCITY;
    }

    //send in NEAR_VELOCITY, FAR_VELOCITY, or custom value
    public double shooterOn(int velocity) {
        ((DcMotorEx) shooter).setVelocity(velocity);
        isShooterOn = true;
        return velocity;
    }

    public double shooterOn(double percentOfMax) {
        if (percentOfMax > 1) {
            percentOfMax = 1;
        } else if (percentOfMax < 0) {
            percentOfMax = 0;
        }
        ((DcMotorEx) shooter).setVelocity(percentOfMax * MAX_VELOCITY);
        isShooterOn = true;
        return percentOfMax * MAX_VELOCITY;
    }

    public boolean isShooterReady(double velocity) {
        return shooter.getVelocity() <= velocity; //our velocities are negative for some reason
    }

    public void shooterOff() {
        ((DcMotorEx) shooter).setVelocity(0);
        isShooterOn = false;
    }

    public void setDeflector(robot.turret) {
        if (robot.turret.inrange) {
            deflector.setPosition(deflectorHeight(robot.turret.distance));
        } else {
            //need to decide what to do if turret is still aligning
        }
    }

    //need to assign buttons to manually adjust deflector in game
    public void raiseDeflectorManual() {

        deflector.setPosition(deflector.getPosition() + .01);

    }

    public void lowerDeflectorManual() {
        deflector.setPosition(deflector.getPosition() - .01);

    }

    public void shootOne() {
        //need code to shoot one ball
        //consider whether we will repeat shootOne or have a shootAll that runs until empty

        shooting = true;
        isDoneShooting=false;

    }

    public void checkMagazine() {
        //code here will use sensors to see if we are done shooting
        isDoneShooting=true;
    }

    public void shootStop() {
        //need code to pause or stop shooting
        shooting = false;
        isDoneShooting=true;
    }

    public void update() {
        setDeflector(robot.turret);
        if (robot.turret.distance <= NEAR && isShooterOn) {
            shooterOn(NEAR_VELOCITY);

        } else if (robot.turret.distance > NEAR && isShooterOn) {
            shooterOn(FAR_VELOCITY);
        }

        if (shooting) {
            checkMagazine();
            if (isDoneShooting) {
                shootStop();
            }
        }

    }
}