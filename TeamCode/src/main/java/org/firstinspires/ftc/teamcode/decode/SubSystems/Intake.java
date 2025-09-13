package org.firstinspires.ftc.teamcode.decode.SubSystems;


      //  import com.qualcomm.robotcore.hardware.CRServo;
       // import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Intake {
    public DcMotorEx intake;

    public boolean isIntakeOn=false;
   // public DistanceSensor extendDistance;
    public DigitalChannel intakeSensor;
    public DigitalChannel stopperSensor;
   // public boolean isIntakeSensor=false;
   // public double extendProx=0;
   public boolean extendProx;
    public boolean intakeProx=true;
    public Servo wrist,catcher,stopper,gripper;
    public ServoImplEx extendo;
    static int stackfive=465;//460
    public double extendoPosition,oldextendoPosition,wristPosition,oldWristPosition;
    public static double MINEXTEND=.23;//.25;//.26;//.29//.06;
    public static double TRANSFEREXTEND=.29;//.27//3;//.29//WAS.31 come in full with new intake
    public static double TRANSFERRELEASE=.27;//.3;
    public static double MAXEXTEND=.73;//.8;//.81'//.5;
    public static double MINWRIST=.73;//.8; //TODO:Set
    public static double MAXWRIST=.84;//.86; //TODO:Set
    public static double WRISTHORIZONTAL=.73;//.83 ;//TODO: Set
    public static double WRISTPICKUP=.84;//.8
    public static double WRISTPICKUPSUB=.82;

    public static double WRISTSPIT=.84;//.8
    public static double WRISTPICKUPTELEOP=.84;// ;
    public static double WRISTCLEAR=.84;
    public static double WRISTTRANSFER=.73;//.74;//.68;//.72;//.41;//.695;
    public static double WRISTAUTO=.73;//.856;
    public static double CATCHOPEN= .1;
    public static double CATCHCLOSE= .5;
    public static double STOPPEROPEN= 0;//TODO Set
    public static double STOPPERCLOSE= 0;//TODO:Set
    public static double FLOOROPEN = .825;//TODO Set
    public static double FLOORCLOSED = .655;//TODO:Set

    public static double EXTENDMULTIPLIER=.01;//.007;
    public static double EXTENDMULTIPLIERSLOW=.0015;
    public boolean rumble=false;
    public boolean stopperClosed=false;
   // private RevColorSensorV3 intakeColor;
   // private RevColorSensorV3 extendDistance;
    public TouchSensor extendTouch;
    public enum IntakeStates{
        OFF, WAIT,
        SPIT, ACQUIRED, IDLE, CLEAR, GETTING, TRANSFER, ONAUTO, GETTINGAUTO, ONBASKET, GETTINGBASKET, SPITBACK, ACQUIREDAUTO, SPITAUTO, ON
    }
    public IntakeStates intakeState = IntakeStates.OFF;

    public enum ExtendoStates{
        IDLE,EXTENDMIN,OUT, CATCH, IN
    }
    public ExtendoStates extendoStates=ExtendoStates.IDLE;
    private ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime extendoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void init(HardwareMap hardwareMap)
    {
      //  intake = hardwareMap.get(CRServo.class, "Intake");
        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
       // intake.setDirection(CRServo.Direction.FORWARD);
       // intakeColor = hardwareMap.get(RevColorSensorV3.class, "IntakeSensor");
     //   extendDistance = hardwareMap.get(RevColorSensorV3.class, "IntakeDist");
       // intakeSensor = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        intakeSensor = hardwareMap.get(DigitalChannel.class, "beambreak");
        intakeSensor.setMode(DigitalChannel.Mode.INPUT);
        stopperSensor = hardwareMap.get(DigitalChannel.class, "optical");
        stopperSensor.setMode(DigitalChannel.Mode.INPUT);
        wrist=hardwareMap.get(Servo.class, "Wrist");
        catcher=hardwareMap.get(Servo.class, "Catcher");
        extendo=hardwareMap.get(ServoImplEx.class, "Extendo");
        gripper=hardwareMap.get(Servo.class, "Gripper");
       extendTouch = hardwareMap.get(TouchSensor.class, "ExtendTouch");
      //  stopper = hardwareMap.get(TouchSensor.class, "Stopper");
      //  extendo.setDirection(Servo.Direction.REVERSE);
      //  extendo.scaleRange(.35,.75);
     /* Moved to AUTO init
        extendoPosition=MINEXTEND;

        setExtendoPos(extendoPosition);
        */

    }

    public void setWristPos(double pos){
        wristPosition=  Range.clip(pos,MINWRIST,MAXWRIST);
    }

    public void wristOut(){
        wristPosition=Range.clip(wristPosition+.005,MINWRIST,MAXWRIST);
    }

    public void wristIn(){
        wristPosition=  Range.clip(wristPosition-.005,MINWRIST,MAXWRIST);
    }

    public void wristPickup(){
        wristPosition = WRISTPICKUP;
    }
    public void wristPickupSub(){
        wristPosition = WRISTPICKUPSUB;
    }
    public void wristPickupTeleop(){
        wristPosition = WRISTPICKUPTELEOP;
    }
    public void wristTransfer(){
        wristPosition = WRISTTRANSFER;
    }
    public void wristAuto(){
        wristPosition = WRISTAUTO;
    }
    public void wristHorizontal(){
        wristPosition = WRISTHORIZONTAL;
    }
    public void wristSpit(){
        wristPosition = WRISTSPIT;
    }
    public void wristClear() {wristPosition=WRISTCLEAR;};
    public void setExtendoPos(double pos){
        extendoPosition=  Range.clip(pos,MINEXTEND,MAXEXTEND);
    }

    public void extendoOut(){
        catchOpen();
        extendoPosition=Range.clip(extendoPosition+.005,MINEXTEND,MAXEXTEND);
    }
    public void extendoOutFast(){
        catchOpen();
        extendoPosition=Range.clip(extendoPosition+EXTENDMULTIPLIER,MINEXTEND,MAXEXTEND);
    }
    public void extendoInFast(){
      //  catchClose();
        extendoPosition=Range.clip(extendoPosition-EXTENDMULTIPLIER,MINEXTEND,MAXEXTEND);
    }

    public void extendoOutSlow(){
        catchOpen();
        extendoPosition=Range.clip(extendoPosition+EXTENDMULTIPLIERSLOW,MINEXTEND,MAXEXTEND);
    }
    public void extendoInSlow(){
        //catchClose();
        extendoPosition=Range.clip(extendoPosition-EXTENDMULTIPLIER,MINEXTEND,MAXEXTEND);
    }

    public void extendoIn(){
       // catchClose();
        extendoPosition=  Range.clip(extendoPosition-.005,MINEXTEND,MAXEXTEND);
    }

    public void extendMin(){
      //  catchClose();
        catchOpen();
        extendoPosition = MINEXTEND;

       // extendoTimer.reset();
       // extendoStates=ExtendoStates.EXTENDMIN;

    }
    public void extendTransfer(){
        oldextendoPosition=extendoPosition;
        extendoPosition = TRANSFEREXTEND;
        if (extendoPosition>oldextendoPosition) {
            catchOpen();
        } else {
            catchClose();
        }








    }
    public void extendFull(){
        catchOpen();
        extendoPosition = MAXEXTEND;
    }

    public void extendMost(){
        catchOpen();
        extendoPosition = MAXEXTEND-.17;
    }

    public void extendMost2(){
        catchOpen();
        extendoPosition = MAXEXTEND-.14;
    }

    public void extendRelease(){
      //  catchClose();
        extendoPosition = TRANSFERRELEASE;
    }

    /*public void extendBasic(){
        extendoPosition = 0.55; //TODO: Set
    }

     */

    public void extendAnalog(double level){
        oldextendoPosition=extendoPosition;
        double scaled = (((level) * (MAXEXTEND - MINEXTEND))) + MINEXTEND;
        double move=Math.signum(scaled)*Math.pow(scaled,2)*EXTENDMULTIPLIER;
        //extendoPosition = 0.55 + 0.05 * level;
        extendoPosition =Range.clip(extendoPosition+move , MINEXTEND,MAXEXTEND);
       if (extendoPosition>oldextendoPosition) {
            catchOpen();
        }
      //  else {catchClose();}

        //  System.out.println("Placing");
    }

    public void extendLevels(int level){
        //extendoPosition = 0.55 + 0.05 * level;
        oldextendoPosition=extendoPosition;
        extendoPosition = MINEXTEND + ((MAXEXTEND-MINEXTEND)/53) * level;
        if (extendoPosition>oldextendoPosition) {
            catchOpen();
        } //else {catchClose();}
        //  System.out.println("Placing");
    }

    public void extendFixed(double pos){
        //extendoPosition = 0.55 + 0.05 * level;
        oldextendoPosition=extendoPosition;
        extendoPosition = Range.clip(pos,MINEXTEND,MAXEXTEND);
        if (extendoPosition>oldextendoPosition) {
            catchOpen();
        } //else {catchClose();}
        //  System.out.println("Placing");
    }
    public void intakeOn(){
        intake.setPower(1);//-1
        isIntakeOn=true;
    }
    public void intakeTransfer() {
        intake.setPower(.65  );//-1
        isIntakeOn=true;
    }
    public void intakeGetting() {
        intake.setPower(.8 );//-1
        isIntakeOn=true;
    }
    public void intakeOnAuto(){
        intake.setPower(1);//-1
        isIntakeOn=true;
    }

    public void intakeOnAuto2(){
        intake.setPower(1);//-1
        isIntakeOn=true;
    }

    public void intakeOff(){
        intake.setPower(0);
        isIntakeOn=false;
    }

    public void intakeSpit(){
        intake.setPower(-1);
        isIntakeOn=false;
        intakeStopperClosed();
    }

    public void intakeStopperClosed() {
        gripper.setPosition(FLOOROPEN);
    }
    public void intakeStopperOpen () {
        gripper.setPosition(FLOORCLOSED);
    }

    public void catchOpen() {
        catcher.setPosition(CATCHOPEN);
    }


    public void catchClose() {
        catcher.setPosition(CATCHCLOSE);
    }

    public void stopperOpen() {
        if (stopperClosed) {
            stopperClosed=false;
            stopper.setPosition(STOPPEROPEN);
        }
    }


    public void stopperClose() {
        if (!stopperClosed) {
            stopperClosed=true;
            stopper.setPosition(STOPPERCLOSE);
        }
    }


    public void getIntakeProx() {
        intakeProx=intakeSensor.getState();
    }

    public void getExtendProx(){
     //   extendProx= ((DistanceSensor) extendDistance).getDistance(DistanceUnit.CM) ;
        extendProx=extendTouch.isPressed();
    }

  /*  public boolean isWrongColor() {
        if (intakeColor.getLightDetected() < 1.5) { //TODO: Set range
            //turnOff();
            return true;
        } else {
            return false;
        }
    }

   */

    public boolean isStopperSensor() {
        return !stopperSensor.getState();

    }

    public boolean isIntakeSensor() {
        return !intakeSensor.getState();
        /*
        if (intakeProx < 1.5) { //TODO: Set range //1.5
            //turnOff();
            return true;
        } else {
            return false;
        }
        *
         */
    }

    public boolean isExtendTouch(){
        boolean retval=false;
    /*    if (extendProx < 5 && extendProx> 3.5) { //TODO: Set range //1.5
            //turnOff();
            return true;
        } else {
            return false;
        }


     */

        if (extendTouch.isPressed() ) {
            // telemetry.addData("Button", "PRESSED");
            retval=true;
        } else {
            // telemetry.addData("Button", "NOT PRESSED");
        }

        return retval;


    }


    public void update() {
        extendoPosition = Range.clip(extendoPosition, MINEXTEND, MAXEXTEND);
        if (extendoPosition<MINEXTEND) {
            if (isExtendTouch()){
                extendoPosition=.4;
                catchOpen();
            }

        }
   //     System.out.println("ExtendPos " + extendoPosition);
        extendo.setPosition(extendoPosition);
        //  extendo.setPosition(extendoPosition);
     /*   if (extendoPosition == oldextendoPosition) {

        } else {
            extendo.setPosition(extendoPosition);
            oldextendoPosition = extendoPosition;
        }


      */
        switch (extendoStates) {
            case IDLE:
                break;
            case EXTENDMIN:
                if (isExtendTouch()){
                    extendoPosition=.5;
                    extendoTimer.reset();
                    extendoStates=ExtendoStates.OUT;
                } else if (extendoTimer.milliseconds()>1000) {
                    extendoStates=ExtendoStates.IDLE;
                }
                break;
            case OUT:
                if (extendoTimer.milliseconds()>1000){
                    catchOpen();
                    extendoTimer.reset();
                    extendoStates=ExtendoStates.CATCH;
                }

                break;
            case CATCH:
                if (extendoTimer.milliseconds()>1500){
                    extendoPosition=MINEXTEND;
                    extendoTimer.reset();
                    extendoStates=ExtendoStates.IDLE;
                }

                break;
            case IN:
                break;
        }

        wristPosition = Range.clip(wristPosition, MINWRIST, MAXWRIST);
        if (wristPosition == oldWristPosition) {

        } else {
            wrist.setPosition(wristPosition);
            oldWristPosition = wristPosition;
        }

        switch (intakeState) {
            case OFF:
              //  if (isIntakeOn) {
                    intakeOff();
                rumble=false;
               // }

                break;
            case ON:
                rumble=false;
              //  stopperClose();
                //if the sensor is triggered, turn off TODO:Manual override? spit if wrong color?
                if (isIntakeSensor()||isStopperSensor()) {
                    rumble=true;
                    intakeTimer.reset();
                    intakeState=IntakeStates.GETTING;

                } else if (!isIntakeOn) //if the intake is off, turn it on
                { intakeOn();}


                break;
            case ONAUTO:
                rumble=false;
                intakeStopperClosed();
             //   stopperClose();
                //if the sensor is triggered, turn off TODO:Manual override? spit if wrong color?
                if (isIntakeSensor()||isStopperSensor()) {
                  //  rumble=true;
                    intakeTimer.reset();
                   // intakeState=IntakeStates.GETTINGAUTO;
                    intakeOff();
                    intakeState=IntakeStates.ACQUIREDAUTO;//skip getting, save a cycle
                } else if (!isIntakeOn) //if the intake is off, turn it on
                { intakeOn();}


                break;
            case ONBASKET:
                rumble=false;
                intakeStopperClosed();
             //   stopperClose();
                //if the sensor is triggered, turn off TODO:Manual override? spit if wrong color?
               if (isStopperSensor()) {
                   rumble=true;
                   intakeOff();
                   intakeStopperOpen();
                   intakeState=IntakeStates.ACQUIRED;
            } else
                if (isIntakeSensor()) {
                    rumble=true;
                    intakeTimer.reset();
                    intakeState=IntakeStates.GETTINGBASKET;

                } else if (!isIntakeOn) //if the intake is off, turn it on
                { intakeOn();}


                break;
            case CLEAR:
                rumble=false;
                getIntakeProx();
                //if the sensor is triggered, turn off TODO:Manual override? spit if wrong color?
             /*   if (isIntakeSensor()) {
                    //intakeOff();
                    wristHorizontal();
                    intakeState=IntakeStates.ACQUIRED;
                } else if (!isIntakeOn) //if the intake is off, turn it on
                { intakeOn();}
*/
                break;
            case GETTING:
                if (intakeTimer.milliseconds()>0) {//was 200
                    intakeState=IntakeStates.ACQUIRED;
                   // intakeGetting();
                    intakeOff();
                }

                break;
            case GETTINGBASKET:
               // intakeTransfer();
//TODO Test the timing - why stop early?
               // if (intakeTimer.milliseconds()>700) {//was 300
                if  ((isStopperSensor())) {//|!isIntakeSensor()&&intakeTimer.milliseconds()>100)||intakeTimer.milliseconds()>450
                    intakeOff();
                    intakeStopperOpen();


                    intakeState=IntakeStates.ACQUIRED;
                   // intakeGetting();
                }
                else if (intakeTimer.milliseconds()>1000) {
                    intakeState=IntakeStates.SPITAUTO;
                    intakeTimer.reset();
                }



                break;
            case GETTINGAUTO:
                if (intakeTimer.milliseconds()>0) {//was 200
                    intakeState=IntakeStates.ACQUIREDAUTO;
                    intakeOff();
                }

                break;
            case ACQUIREDAUTO:
                intakeOff();
                isIntakeOn=false;
                wristSpit();


                break;
            case ACQUIRED:
                intakeOff();
                intakeStopperOpen();
                isIntakeOn=false;
                wristHorizontal();


                break;
            case TRANSFER:
               // intakeTransfer();
                break;
            case IDLE:
                if (isIntakeOn) {
                    intakeOff();
                }
                rumble=false;
                break;
            case SPIT:
                rumble=false;
                intakeStopperClosed();
                intakeSpit();
                break;
            case SPITAUTO:
                rumble=false;
                intakeStopperClosed();
                intakeSpit();
                if (intakeTimer.milliseconds()>500){
                    intakeState=IntakeStates.ONBASKET;
                }
                break;
            case SPITBACK:
                rumble=false;
                intakeStopperClosed();
                intakeOn();
                break;
        }

    }
    }

