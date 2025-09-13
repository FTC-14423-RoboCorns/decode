package org.firstinspires.ftc.teamcode.decode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.FTCLibPIDFController;

@Config
public class Lift {
    private FTCLibPIDFController pidController;
    public DcMotorEx liftRear;
    public DcMotorEx liftFront;
    public boolean isHang = false;
    public DcMotorEx liftHang;


    public boolean isPlacing = false;

    private static double ADJUSTTICKS=312/1150;
    private static int MAXLIFT=787;//2900*ADJUSTTICKS ; //was 2800
    private static int MINLIFT=0;
    private static int MAXHANGLIFT=5100; //TODO:Set min and max
    private static int MINHANGLIFT=0;
    public static int LIFTTRANSFERAUTO=150;
    private static int LIFTTRANSFER=170;//200*ADJUSTTICKS;//400;//54
    private static int LIFTPLACE=120;//30;//100;//135;//145;//500*ADJUSTTICKS;
    private static int LIFTPLACEAUTO=90;
    ///, 135 for normal
    private static double KP=0.05; //0.5 //TODO:Set PID
    private static double KI=0; //0.001
    private static double KD=0;
    private static int MINPOWER=0;
    private static int MAXPOWER=1;
    private static int LIFTHANG=5100;  //TODO: Set Lift Hang
    private static int LIFTHANGPREP=1475;  //TODO: Set Lift Hang
    private static int LIFTHANGDOWN=3500;
    private static int TOLERANCE=20;//TODO set tolerance
    private static int BASKETTOLERANCE=27;//100*ADJUSTTICKS;
    private boolean goingDown;

    public enum LiftLevel {//TODO: set lift values
        SPECIMENHIGH(210),//600*ADJUSTTICKS//80 for straight in, 160 for lift up
        SPECIMENLOW(81),//300*ADJUSTTICKS
        BASKETHIGH(720),//2650*ADJUSTTICKS //660//720
        BASKETLOW(190),//700*ADJUSTTICKS
        HANG(407);//1500*ADJUSTTICKS
        private final int liftvalue;

        LiftLevel(final int newValue) {
            liftvalue = newValue;
        }

        public int getValue() { return liftvalue;}
        private static LiftLevel[] vals = values();
        public LiftLevel next(){
       //     System.out.println("In next "+this.ordinal()+ " "+vals.length);
       //     System.out.println("In next new"+vals[(this.ordinal()-1+vals.length) % vals.length]);
       //     System.out.println("In next vals"+vals);
            return vals[(this.ordinal()+1)% vals.length];

        }
        public LiftLevel previous(){
        //    System.out.println("In prev "+this.ordinal()+ " "+vals.length);
        //    System.out.println("In prev new"+vals[(this.ordinal()-1+vals.length) % vals.length]);
            return vals[(this.ordinal()-1+vals.length) % vals.length];


        }
    }
    public LiftLevel liftLevel=LiftLevel.BASKETLOW;

    public int targetPosition, oldtargetPosition,targetHangPosition;
    //public double extendoPosition,oldextendoPosition;
    double rawPID,liftPower,liftHangPower,oldliftPower;
    TouchSensor liftTouch;

    public void init(HardwareMap hardwareMap)
    {
       liftFront = hardwareMap.get(DcMotorEx.class, "liftfront");
        liftRear = hardwareMap.get(DcMotorEx.class, "liftrear");
       // extendo=hardwareMap.get(Servo.class, "ExtendoServo");

        liftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        liftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftHang = hardwareMap.get(DcMotorEx.class, "lifthang");
        liftHang.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRear.setTargetPositionTolerance(5);
        liftFront.setTargetPositionTolerance(5);
        liftRear.setPositionPIDFCoefficients(20);
        liftFront.setPositionPIDFCoefficients(20);
        liftHang.setTargetPosition(liftHang.getCurrentPosition());
        pidController = new FTCLibPIDFController(KP, KI, KD, 0, 0, 0);
        pidController.setIntegrationBounds(-1, 1);
        pidController.setTolerance(20);//TODO:set tolerance
        liftHangPower=0;
        //targetPosition=0; TODO: init in auto/teleop
        //extendoPosition=0;
        liftTouch = hardwareMap.get(TouchSensor.class, "Lift");//TODO:put correct name in
      //  digitalTouch.setMode(RevTouchSensor.Mode.INPUT);
    }

    //TODO: Set increments for lift
    public void liftUp(){
        targetPosition=Range.clip(targetPosition+(14),MINLIFT,MAXLIFT);//50*ADJUSTTICKS
        liftPower=1;
    }

    public void liftHangUp(){
        targetHangPosition=Range.clip(targetHangPosition+(14),MINHANGLIFT,MAXHANGLIFT);//50*ADJUSTTICKS
        liftHangPower=1;
    }

    public void liftPlace(){
        targetPosition=Range.clip(targetPosition+LIFTPLACE  ,MINLIFT,MAXLIFT);
        liftPower=1;
    }

    public void liftPlaceAuto(){
        targetPosition=Range.clip(targetPosition+LIFTPLACEAUTO  ,MINLIFT,MAXLIFT);
        liftPower=1;
    }

    public boolean targetSpecimen(){
       return (liftLevel== LiftLevel.SPECIMENHIGH||liftLevel== LiftLevel.SPECIMENLOW);
    }

    public void setIsHang(boolean b){
        isHang = b;
    }

    //TODO: Integrate touch sensor
    public void liftDown(){
      //  targetPosition=  Range.clip(targetPosition-50,MINLIFT,MAXLIFT);
        targetPosition-=50;//no need for range check with the button
        liftPower=1;
    }
    public void liftHangDown(){
        //  targetPosition=  Range.clip(targetPosition-50,MINLIFT,MAXLIFT);
        targetHangPosition=Range.clip(targetHangPosition-50,MINHANGLIFT,MAXHANGLIFT);
        //targetHangPosition-=50;//no need for range check with the button
        liftHangPower=1;
    }

    public void liftHang(){
        targetHangPosition = LIFTHANG;
        liftHangPower=1;
    }
    public void liftHangPrep(){
        targetHangPosition = LIFTHANGPREP;
        liftHangPower=1;
    }

    public void liftPullUp(){
        targetPosition = 200;
        liftPower = 1;
    }

    public void liftTransfer(){
        targetPosition = LIFTTRANSFER;
        liftPower = 1;
    }

    public void liftTransferAuto(){
        targetPosition = LIFTTRANSFERAUTO;
        liftPower = 1;
    }

    public void liftHangDone(){
     //   liftHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   liftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetHangPosition = LIFTHANGDOWN;
        liftHang.setPower(1);
      //  liftFront.setPower(-1);
    }

    public void setLiftPos(int pos){
        targetPosition=  Range.clip(pos,MINLIFT,MAXLIFT);
        liftPower=1;
    }

    public void setLiftHangPos(int pos){
        targetHangPosition=  Range.clip(pos,MINHANGLIFT,MAXHANGLIFT);
        liftHangPower=1;
    }
/*
    public void setExtendoPos(int pos){
        extendoPosition=  Range.clip(pos,MINEXTEND,MAXEXTEND);
    }

    public void extendoOut(){
        extendoPosition=Range.clip(extendoPosition+.05,MINEXTEND,MAXEXTEND);
    }

    public void extendIn(){
        extendoPosition=  Range.clip(extendoPosition-.05,MINEXTEND,MAXEXTEND);
    }

    public void extendMin(){
        extendoPosition = 0;
    }
    public void extendFull(){
        extendoPosition = 0.7;
    }

    public void extendBasic(){
        extendoPosition = 0.55;
    }

    public void extendLevels(int level){
        //extendoPosition = 0.55 + 0.05 * level;
        extendoPosition = 0.2 + 0.05 * level;
      //  System.out.println("Placing");
    }
*/
  //TODO: Set levels, set ENUM for levels
    public void liftLevel(int level){
        if(level == 1){
            targetPosition = Range.clip(700, MINLIFT, MAXLIFT); //was 600
            //liftPower=1;
        }
        else {
            targetPosition = Range.clip(700 + (level-1) * 260, MINLIFT, MAXLIFT); //was level*400
           /// liftPower = 1;
        }
    }
/*
    public void extendTOBoard(double distanceAway){
        if(!isPlacing){
            if(distanceAway > 20.5){
                extendoPosition += 0.025;
            }
            if(distanceAway < 16.5){
                extendoPosition -= 0.025;
            }
        }
    }

    public void extendTOBoardAuto(double distanceAway){
        if(!isPlacing){
            if(distanceAway > 22.5){
                extendoPosition += 0.03;
            }
            if(distanceAway < 20.5){
                extendoPosition -= 0.03;
            }
        }
    }

    public void placeBoard(){
        extendoPosition += 0.03;
        isPlacing = true;
    }

    public void placeAuto(){
        extendoPosition += 0.025;
        isPlacing = true;
    }

    public void returnBoard(){
        extendoPosition -= 0.05;
        isPlacing = false;
    }

    public void returnFast(){
        extendoPosition -= 0.2;
        isPlacing = false;
    }

    */
    public void liftAuto(){

            targetPosition = Range.clip(500, MINLIFT, MAXLIFT);
            liftPower=1;

    }

    public boolean isAtPosition(){
        return Math.abs(liftRear.getCurrentPosition()-targetPosition)<=TOLERANCE;

    }

    public boolean isAtBasketPosition(){
        return Math.abs(liftRear.getCurrentPosition()-targetPosition)<=BASKETTOLERANCE;

    }


    public boolean isDown(){
        boolean retval=false;
        if (liftTouch.isPressed() ) {
           // telemetry.addData("Button", "PRESSED");
            retval=true;
        } else {
           // telemetry.addData("Button", "NOT PRESSED");
        }

        return retval;
    }

    public void liftFullDown(){
        goingDown=true;
        liftPower=-.25;
    }

    public void liftOff(){liftPower=0;}
    public void update() {
        if(!isHang) {


                if (goingDown) {
                    if (isDown() ) {
                        resetLift();
                    } else {
                        liftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        liftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        liftRear.setPower(liftPower);
                        liftFront.setPower(liftPower);

                    }
                } else {
                    if (isDown() && (targetPosition <= liftRear.getCurrentPosition())) {
                        resetLift();
                    } else  if (targetPosition == oldtargetPosition) {

                    }  else {
                      if  (targetPosition+500 <= (liftRear.getCurrentPosition())) {
                          liftRear.setPositionPIDFCoefficients(5);
                          liftFront.setPositionPIDFCoefficients(5 );
                        } else {
                          liftRear.setPositionPIDFCoefficients(20);
                          liftFront.setPositionPIDFCoefficients(20);
                        }
                        liftRear.setTargetPosition(targetPosition);
                        liftFront.setTargetPosition(targetPosition);
                        liftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        liftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        // pidController.reset();
                        // pidController.setSetPoint(targetPosition);
                        oldtargetPosition = targetPosition;
                    }
                    // liftrear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    // liftfront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

/*
                rawPID = pidController.calculate(liftFront.getCurrentPosition());

                liftPower = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), MINPOWER, MAXPOWER);


 */

                 //   System.out.println("14423 liftRearPos "+liftRear.getCurrentPosition());
                 //   System.out.println("14423 liftFrontPos "+liftFront.getCurrentPosition());
                    liftRear.setPower(liftPower);
                    liftFront.setPower(liftPower);

                }
            /*
            //only change the power if the change is more than 1 percent. Play with this - esp for auto
            if (liftPower==0||(Math.abs(liftPower - oldliftPower)>Math.abs(.01*liftPower))) {
                liftRear.setPower(liftPower);
                liftFront.setPower(liftPower);
            }

             */
        } else {
            liftHang.setTargetPosition(targetHangPosition);
            liftHang.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftHang.setPower(liftHangPower);
            if (targetPosition == oldtargetPosition) {

            } else if (isDown() && (targetPosition <= liftRear.getCurrentPosition())) {
                resetLift();
            } else {

                liftRear.setTargetPosition(targetPosition);
                liftFront.setTargetPosition(targetPosition);
                liftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                // pidController.reset();
                // pidController.setSetPoint(targetPosition);
                oldtargetPosition = targetPosition;
            }
            // liftrear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            // liftfront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

/*
                rawPID = pidController.calculate(liftFront.getCurrentPosition());

                liftPower = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), MINPOWER, MAXPOWER);


 */

            liftRear.setPower(liftPower);
            liftFront.setPower(liftPower);
          //  liftFront.setPower(liftRear.getPower());
        }
            /*
            extendoPosition=Range.clip(extendoPosition,MINEXTEND,MAXEXTEND);
          //  extendo.setPosition(extendoPosition);
           if (extendoPosition==oldextendoPosition) {

            } else {
                extendo.setPosition(extendoPosition);
                oldextendoPosition=extendoPosition;
            }
           // System.out.println("extendoPosition: " + extendoPosition);
        } else {
            liftHangDown();
        }

             */


    }


    public void resetLift() {
        liftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition=0;
        goingDown=false;
        oldtargetPosition=0;
        liftPower=0;
        liftRear.setPower(liftPower);
        liftFront.setPower(liftPower);
    }

}

