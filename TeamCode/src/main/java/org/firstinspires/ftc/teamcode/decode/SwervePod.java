package org.firstinspires.ftc.teamcode.decode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.FTCLibPIDFController;

@Config
public class SwervePod {
    private FTCLibPIDFController pidController;
    private FTCLibPIDFController drivePIDController;
    public AnalogInput axonencoder;
    private CRServo servo;
    private boolean isDriving = false;
    public DcMotorEx driveMotor;
    private double scaleSpeed = 1;
    private boolean isAligned;
    private boolean isFlipped =false;
    public static double KP=0.55 ;//.6//0.8; //0.5
    public static double KI=.005; //0.001
    public static double KD=.03;
    public static double MIN=0.07;
    public static double MAX=1;
    private double offset;
    public boolean init=false;
    double newservopower,newmotorpower,oldservopower,oldmotorpower=0;
    public double podError;
    String podname;
    double targetAngle=0.0;
     double curAngle, rawPID,servoPower;
    public SwervePod(HardwareMap hardwareMap, String name, String servoname, String motorName, boolean reversed,double off) {
        driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        axonencoder = hardwareMap.get(AnalogInput.class, name);
        servo = hardwareMap.get(CRServo.class, servoname);
        podname = servoname;
        offset =off;
        MotorConfigurationType motorConfigurationType = driveMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        driveMotor.setMotorType(motorConfigurationType);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (reversed) {
            driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        pidController = new FTCLibPIDFController(KP, KI, KD, 0, 0, 0);
        pidController.setIntegrationBounds(-1, 1);
        pidController.setTolerance(Math.toRadians(2));

        drivePIDController = new FTCLibPIDFController(0.4, 0, 0, 0, 0, 0);
        drivePIDController.setIntegrationBounds(-1, 1);
        drivePIDController.setTolerance(Math.toRadians(2));

    }

    public void reversemotor(){
        driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }




    public double getError(){
        return podError;
    }

    public void initTurnTo(double angle){
        //  scaleSpeed = scaleSpeed(angle);

            pidController.setTolerance(Math.toRadians(5));

        if (angle!=targetAngle){
            targetAngle=angle;
            pidController.reset();
            drivePIDController.reset();
        }
        curAngle=readAngle();
        double error = angle - curAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        //   System.out.println("14423"+podname);
        //   System.out.println("14423 initial error "+error);
        //    System.out.println("14423 NormDelta error "+Angle.normDelta(angle-curAngle));
     /*   if (Math.abs(error)>Math.toRadians(10)) {
            scaleSpeed=0;
        } else if (Math.abs(error)>Math.toRadians(4)){
            scaleSpeed=.3;
        } else {
            scaleSpeed=1;
        }

      */

        error=flip(error);
        podError = error;
        /*if (isFlipped){
            angle=Math.abs(angle-Math.toRadians(180));
        }*/


        //setPIDAngle(angle); //CHANGED
        setPIDAngle(0.0);//now using error for PID

        if(isDriving){
            rawPID=drivePIDController.calculate(podError);
            servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),0,MAX);
            if (drivePIDController.atSetPoint()){
                servoPower=0.0;
                drivePIDController.reset();

            }
        }
        else{
            rawPID=pidController.calculate(podError);
            servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),MIN,MAX);
            if (pidController.atSetPoint()){
                servoPower=0.0;
                pidController.reset();

            }
        }
        //*-1



        //    System.out.println("14423"+"Angle " + Math.toDegrees(curAngle));
        //     System.out.println("14423"+"target Angle "+Math.toDegrees(targetAngle));
        //     System.out.println("14423"+"flipped " + isFlipped);
        //     System.out.println("14423"+"flip Angle " + Math.toDegrees(angle));
        //      System.out.println("14423"+"pid target " +Math.toDegrees(pidController.getSetPoint()));

        //  System.out.println("14423"+"pid error "+Math.toDegrees(pidController.getPositionError()));
        //  System.out.println("14423"+"current error "+Math.toDegrees(podError));
        // System.out.println("14423"+"servo power " +servoPower);

        newservopower=servoPower;
        if (newservopower==oldservopower) {
        } else {
            servo.setPower(newservopower*-1);
            oldservopower=newservopower;
        }
        // servo.setPower(servoPower);
//servo.setPower(0.0);



        // OLD PID
/*
        if (Math.abs(error) < Math.toRadians(2)){//increased from 1
            servo.setPower(0);
        }
        else{
            servo.setPower(turnPID(error));//reversed for IMU change?
        }

 */
        //  System.out.println("14423"+"servoangle error "+podname + Math.toDegrees(angle));
        /*
        System.out.println("14423 servopower"+podname+servo.getPower());
        System.out.println("14423 servoangle"+podname+Math.toDegrees(curAngle));


        System.out.println("14423"+"servoangle error "+podname + Math.toDegrees(error));


         */

        //System.out.println("14423"+"pid error "+Math.toDegrees(error));

        /*System.out.println("14423"+"Angle " + Math.toDegrees(curAngle));
        System.out.println("14423"+"flipped " + isFlipped);
        System.out.println("14423"+"flip Angle " + Math.toDegrees(angle));
        System.out.println("14423"+"pid target " +Math.toDegrees(pidController.getSetPoint())); */



    }
    public void turnTo(double angle){
      //  scaleSpeed = scaleSpeed(angle);

            pidController.setTolerance(Math.toRadians(2));

        if (angle!=targetAngle){
            targetAngle=angle;
            pidController.reset();
            drivePIDController.reset();
        }
        curAngle=readAngle();
        double error = angle - curAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
     //   System.out.println("14423"+podname);
     //   System.out.println("14423 initial error "+error);
    //    System.out.println("14423 NormDelta error "+Angle.normDelta(angle-curAngle));
     /*   if (Math.abs(error)>Math.toRadians(10)) {
            scaleSpeed=0;
        } else if (Math.abs(error)>Math.toRadians(4)){
            scaleSpeed=.3;
        } else {
            scaleSpeed=1;
        }

      */

        error=flip(error);
        podError = error;
        /*if (isFlipped){
            angle=Math.abs(angle-Math.toRadians(180));
        }*/


       //setPIDAngle(angle); //CHANGED
        setPIDAngle(0.0);//now using error for PID

        if(isDriving){
            rawPID=drivePIDController.calculate(podError);
            servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),0,MAX);
            if (drivePIDController.atSetPoint()){
                servoPower=0.0;
                drivePIDController.reset();

            }
        }
        else{
            rawPID=pidController.calculate(podError);
            servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),MIN,MAX);
            if (pidController.atSetPoint()){
                servoPower=0.0;
                pidController.reset();

            }
        }
        //*-1



   //    System.out.println("14423"+"Angle " + Math.toDegrees(curAngle));
   //     System.out.println("14423"+"target Angle "+Math.toDegrees(targetAngle));
   //     System.out.println("14423"+"flipped " + isFlipped);
   //     System.out.println("14423"+"flip Angle " + Math.toDegrees(angle));
    //      System.out.println("14423"+"pid target " +Math.toDegrees(pidController.getSetPoint()));

    //  System.out.println("14423"+"pid error "+Math.toDegrees(pidController.getPositionError()));
    //  System.out.println("14423"+"current error "+Math.toDegrees(podError));
   // System.out.println("14423"+"servo power " +servoPower);

        newservopower=servoPower;
        if (newservopower==oldservopower) {
        } else {
            servo.setPower(newservopower*-1);
            oldservopower=newservopower;
        }
       // servo.setPower(servoPower);
//servo.setPower(0.0);



        // OLD PID
/*
        if (Math.abs(error) < Math.toRadians(2)){//increased from 1
            servo.setPower(0);
        }
        else{
            servo.setPower(turnPID(error));//reversed for IMU change?
        }

 */
        //  System.out.println("14423"+"servoangle error "+podname + Math.toDegrees(angle));
        /*
        System.out.println("14423 servopower"+podname+servo.getPower());
        System.out.println("14423 servoangle"+podname+Math.toDegrees(curAngle));


        System.out.println("14423"+"servoangle error "+podname + Math.toDegrees(error));


         */

        //System.out.println("14423"+"pid error "+Math.toDegrees(error));

        /*System.out.println("14423"+"Angle " + Math.toDegrees(curAngle));
        System.out.println("14423"+"flipped " + isFlipped);
        System.out.println("14423"+"flip Angle " + Math.toDegrees(angle));
        System.out.println("14423"+"pid target " +Math.toDegrees(pidController.getSetPoint())); */



    }

    public void setMotorPower(double power){
        newmotorpower=power * scaleSpeed;
        if (newmotorpower==oldmotorpower) {

        } else {
            driveMotor.setPower(newmotorpower);
            oldmotorpower=newmotorpower;
         //   System.out.println("14423 motorpower"+podname+newmotorpower);
         //   System.out.println("14423 flipped"+podname+isFlipped);
         //   System.out.println("14423 direction"+podname+driveMotor.getDirection());
        }
            if(newmotorpower > 0.1){
                isDriving = true;
            }
            else{
                isDriving = false;
            }

    }
    public double turnPID(double error){


        if(error > Math.toRadians(20)){//40
            return .01;
        }
        else if(error > Math.toRadians(4)){
            return Math.toDegrees(error)/320;//80
        }
        else if (error > 0){
            return .01;//0.07;
        }
        else if(error < Math.toRadians(-20)){//-40
            return -.01;
        }
        else if (error < Math.toRadians(-4)){
            return Math.toDegrees(error)/320;//80
        }
        else{
            return -.01;//-0.07;
        }
    }

    public double flip(double error) {
        if(Math.abs(error) > Math.toRadians(90)){
        //    isFlipped=true;
           if(driveMotor.getDirection().equals(DcMotorSimple.Direction.FORWARD)){
                driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else{
                driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            isFlipped = !isFlipped;
           error -=Math.toRadians(180);
            if(error > Math.toRadians(180)){
                error -= Math.toRadians(360);
            }
            else if(error < -Math.toRadians(180)){
                error += Math.toRadians(360);
            }


        }
        return error;
        /*else {
            isFlipped=false;
        }*/
    }

    public void setPIDAngle(Double angle){
       // target=angle;

        if (!(angle==pidController.getSetPoint())) {
            //System.out.println("setting new point "+ Math.toDegrees(angle));
            pidController.setSetPoint(angle);
            // update();
        }

    }
    // divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
    // double position = analogInput.getVoltage() / 3.3 * 360;
    //TODO: Add offset
    public double readAngle(){
        if(isFlipped){
           // 360 - (((LeftFrontEncoder.getVoltage() / 3.3) * 360 - 195) + 360) % 360
            return Math.toRadians(360)- ((((axonencoder.getVoltage()*Math.toRadians(360)) / 3.3)-offset + Math.toRadians(360) + Math.toRadians(180)) % Math.toRadians(360));
        }
        else{
            return  Math.toRadians(360)- ((((axonencoder.getVoltage()*Math.toRadians(360)) / 3.3)-offset+ Math.toRadians(360))% Math.toRadians(360));
        }
    }

    public double getEncoderPos(){
        return driveMotor.getCurrentPosition();
    }
    public double getVelocity(){return driveMotor.getVelocity(); }
    public double scaleSpeed(double angle){
        double error =  angle - readAngle();
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        /*if (isAuto){//
            return 1;
        }
        else if (Math.abs(error) > Math.toRadians(10) && !isAuto){
            return 1;
        }
        else{
            return 1;
        }*/
        return 1;
    }
}
