package org.firstinspires.ftc.teamcode.decode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Arm {
    private double leftPos = 0.515;// init to .515
    double oldleftPos;
    private double rightPos = 0.31;
    public Servo claw;
    public Servo leftServo;

    public ServoImplEx armServo;
    public Servo armStopper;
    public ServoImplEx clawWrist;
    public static double CLAWOPEN= .75;// .83;//0;
    public static double CLAWCLOSED=.41;//
    public static double CLAWCLOSEDSPECIMEN=.39;//
    public static double CLAWCLOSEDBBASKET=.41;//.62;//// .45;//.2;
    public static double CLAWTRANSFER=.83;
    public static double MINCLAWWRIST=0; //TODO:Set
    public static double MAXCLAWWRIST=.9; //TODO:Set
    //public static double CLAWWRISTPICKUP=.23;
    //public static double CLAWWRISTTRANSFER=.23;
    //public static double CLAWWRISTDUMP=.27;
    //public static double CLAWWRISTINTAKE=.23;//TODO:set
    //public static double CLAWWRISTBASKET=.6;//TODO:set
    public static double CLAWWRISTFRONT=.85;
    public static double CLAWWRISTBACK=.1;
    public static double CLAWWRISTOVER=0;
    public static double CLAWWRISTAUTO=.68;
    public static double CLAWWRISTRIGHT=.45;
    public static double STOPPEROPEN=.8;
    public static double STOPPERCLOSED=.5;
    public static double ARMPICKUP=0;
    public static double ARMTRANSFER=1;
    public static double ARMTRANSFERREADY = .8  ;
    public static double ARMPLACE=.74;//.86;//.72;//.72;
    //.82 for straight in, .74 for lift, .76 for no lift straight
    public static double ARMPREBASKET=.5;
    public static double ARMPLACEDONE=.6;
    public static double ARMAUTO=.98;
    public static double ARMBASKET=.66 ;//.39;//.4;//.5 //.35
    public static double ARMBACKBASKET=.43;//.39 ;
    public static double ARMBASKETDROP=.72 ;
    public static double ARMBASKETAUTO=.31;
    public static double ARMBASKETPARK=.30; //ALSO for park
    public static double WAIT=35;
    public static double INCREMENT=.025;
    private static double MAXARMVEL=INCREMENT/WAIT; //speed in position per ms
double turnPos;
    public Servo turner;
    public double stopperPosition,oldWristPosition,clawPosition, armTargetPosition, oldArmTargetPosition,initialArmPosition,armTargetError,interimArmTarget =0;
   // public double wristPosition,oldWristPosition,armPosition,oldArmPosition;
    private ElapsedTime armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double curTime,lastTime;
    boolean manual =false;
    public void init(HardwareMap hardwareMap){
        armServo = hardwareMap.get(ServoImplEx.class, "rotator");
       // rightServo = hardwareMap.get(Servo.class, "WristRightServo");
      //  rightServo.setDirection(Servo.Direction.REVERSE);
        armServo.setPwmRange(new PwmControl.PwmRange(500,2500));
       // armServo.setDirection(ServoImplEx.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        armStopper = hardwareMap.get(Servo.class, "clawrotate");
        clawWrist = hardwareMap.get(ServoImplEx.class, "armrotate");
        armTimer.startTime();
        lastTime=armTimer.milliseconds();
        armTargetPosition=ARMBASKET;
       oldArmTargetPosition=ARMBASKET;
        stopperPosition = STOPPEROPEN;
        clawPosition=CLAWWRISTFRONT;
     //   armServo.setPosition(armTargetPosition);
     //   initialArmPosition=armServo.getPosition();
     //   armTargetPosition=initialArmPosition ;
      //  oldArmTargetPosition=initialArmPosition;
     //  wristPosition=clawWrist.getPosition();

    }


    public void setClawWristPos(int pos){
        clawPosition =  Range.clip(pos,MINCLAWWRIST,MAXCLAWWRIST);
    }

    public void clawWristFront(){
        clawPosition =Range.clip(CLAWWRISTFRONT,MINCLAWWRIST,MAXCLAWWRIST);
    }
    public void clawWristAuto(){
        clawPosition =  Range.clip(CLAWWRISTAUTO,MINCLAWWRIST,MAXCLAWWRIST);
    }
    public void clawWristBack(){
        clawPosition =  Range.clip(CLAWWRISTBACK,MINCLAWWRIST,MAXCLAWWRIST);
    }
    public void clawWristOver(){
        clawPosition =  Range.clip(CLAWWRISTOVER,MINCLAWWRIST,MAXCLAWWRIST);
    }
    public void clawWristRight(){
        clawPosition =  Range.clip(CLAWWRISTRIGHT,MINCLAWWRIST,MAXCLAWWRIST);
    }

    public void openStopper() {
        stopperPosition = STOPPEROPEN;}
    public void closeStopper() {
        stopperPosition = STOPPERCLOSED;}

  /*  public void clawWristPickup(){
        wristPosition = CLAWWRISTPICKUP;
    }
    public void clawWristTransfer(){
        wristPosition = CLAWWRISTTRANSFER;
    }

    public void clawWristDump(){
        wristPosition = CLAWWRISTDUMP;
    }


    public void clawWristIntake() {
        wristPosition = CLAWWRISTINTAKE;
    }
    public void clawWristBasket() {
        wristPosition = CLAWWRISTBASKET;
    }

   */
    public void wristLift(){leftPos = 0.38;}
    /*public void turnerRightTurn(){
        turner.setPosition(0.85);
    }

    public void turnerLeftTurn(){
        turner.setPosition(0.15);
    }

    public void turnerReturn(){
        turner.setPosition(0.5);
    }



    public void turnerAuto(double robotAngle){
        turnPos=.5-((Math.toDegrees(robotAngle)-270)/SIDETURNANGLEADJUST);
        turner.setPosition(turnPos);
    }


     */

    public void setArmPos(int pos){
        armTargetPosition =  Range.clip(pos,ARMPICKUP,ARMTRANSFER);
        manual=false;
    }

    public void armFront(){
        armTargetPosition =Range.clip(armTargetPosition +.01,ARMPICKUP,ARMTRANSFER);
        manual=true;
    }

    public void armRear(){
        armTargetPosition =  Range.clip(armTargetPosition -.01,ARMPICKUP,ARMTRANSFER);
        manual=true;
    }

    public void armPickup(){
        armTargetPosition = ARMPICKUP;
        manual=false;
    }
    public void armTransferGetReady(){
        armTargetPosition = ARMTRANSFERREADY;
        manual=false;
    }
    public void armTransfer(){
        armTargetPosition = ARMTRANSFER;
        manual=false;
    }
    public void armPlace(){
        armTargetPosition = ARMPLACE;
        manual=false;
    }
    public void armPreBasket(){
        armTargetPosition = ARMPREBASKET;
        manual=false;
    }
    public void armAuto(){
        armTargetPosition = ARMAUTO;
        manual=false;
    }

    public void armPlaceDone(){
        armTargetPosition = ARMPLACEDONE;
        manual=false;
    }
    public void armBasket(){
        armTargetPosition = ARMBASKET;
        manual=false;
    }

    public void armBackBasket(){
        armTargetPosition = ARMBACKBASKET;
        manual=false;
    }

    public void armBasketDrop(){
        armTargetPosition = ARMBASKETDROP;
        manual=false;
    }

    public void armBasketAuto(){
        armTargetPosition = ARMBASKETAUTO;
        manual=false;
    }

    public void armBasketPark(){
        armTargetPosition = ARMBASKETPARK;
        manual=false;
    }

    public void closeClaw(){
        claw.setPosition(CLAWCLOSED  );

    }
    public void closeClawSpecimen(){
        claw.setPosition(CLAWCLOSEDSPECIMEN  );

    }
    public void closeClawBasket(){
        claw.setPosition(CLAWCLOSEDBBASKET  );
    }
    public void transferClaw(){
        claw.setPosition(CLAWTRANSFER  );
    }

    public boolean isArmAtPickup() {
        return armServo.getPosition()==ARMPICKUP;
    }
    public boolean isArmAtTransfer() {
        return armServo.getPosition()==ARMTRANSFER;
    }
    public boolean isArmAtPlace() {
        return armServo.getPosition()==ARMPLACE;
    }
    public boolean isArmAtPlaceDone() {
        return armServo.getPosition()==ARMPLACEDONE;
    }
    public boolean isArmAtTransferGetReady() {
        return armServo.getPosition()==ARMTRANSFERREADY;
    }
    public boolean isArmAtBasket() {
        return armServo.getPosition()==ARMBASKET;
    }

    public void openClaw(){
        claw.setPosition(CLAWOPEN);
        }

    public double getArmTarget(double armPos){
        double retval=0;
        curTime=armTimer.milliseconds();
        if (curTime-lastTime>WAIT) {
         //   System.out.println("14423arm target value "+armPos);
            double diffPos = armPos-armServo.getPosition();
            retval=Math.signum(diffPos) * Range.clip(Math.abs(diffPos), 0, INCREMENT);
            lastTime=curTime;
        }

            return retval;

    }

    public void update(){
//       rightServo.setPosition(rightPos);
     /*   if (wristPosition==oldWristPosition) {
        } else{
            clawWrist.setPosition(wristPosition);
            oldWristPosition=wristPosition;
        }

      */
        armStopper.setPosition(stopperPosition);
          //  if (manual) {
            /*    double move = getArmTarget(armTargetPosition);

      //      System.out.println("14423arm move "+move);
            if (Math.abs(move)>0) {
         armServo.setPosition(Range.clip(armServo.getPosition()+move,ARMPICKUP,ARMTRANSFER));

            //    System.out.println("14423arm servo "+armServo.getPosition());

            }

             */
        clawWrist.setPosition(clawPosition);
        armServo.setPosition(Range.clip(armTargetPosition,ARMPICKUP,ARMTRANSFER));
            /*else {armServo.setPosition(Range.clip(armTargetPosition,ARMPICKUP,ARMTRANSFER));}
        }

             */


    }
}
