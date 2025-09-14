package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.decode.DecodeRobot;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Data;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.HashMap;
import java.util.Map;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

//TODO: Test with fieldcentric off to verify linearvelocity
//TODO: Block Test servo.setdirection and angles to test wheel angle
    //set direction does not affect angle readings

    //TODO: Consider sign on both left stick x and right stick x
    //TODO: Consider field centric rotation angle (270 to 90) on manualyaw reset
//   TODO: Test rotation angles for servo angles for rotation and make consistent w/direction
//angles for turning appear correct, unclear if motor direction is correct
    //TODO: Add localizer and test

@TeleOp(group = "drive")
public class MainTeleOp extends OpMode {
    public DecodeRobot robot = new DecodeRobot(0,0, 0);
    public double angle = 0 ;
    public int level =1;
    //public boolean isIntakeOn = false;
    public ElapsedTime liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime dropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime openTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime ledTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    HashMap<String, Boolean> buttonTable;
    private boolean fieldCentric=true;
    private double extend=0.0;
    private double multiplier=1;
    private boolean ledOn=true;
double prox=30;
    private int isBlue = 1;

    private double deadx,deady,deadrot;
    private double deadBand=.2;
    private boolean autodrive=false;

    public enum LEDStates{
        EMPTY,
        PICKUP,
        PLACE, CHANGETARGET, CHANGESIDE, PIXEL, INTAKE
    }

    public LEDStates ledState = LEDStates.EMPTY;
    public LEDStates oldledState= LEDStates.EMPTY;
    public enum TransferStates {
        OPEN,
        PICKUP,
        DONE, FINISHPLACE, HALFDONE, STARTDROPOFF, DROPOFF, STARTPLACE, HALFRETRACT, IDLE, STARTPICKUP, STARTTRANSFER, TRANSFER, ALIGNTRANSFER, RESET, CLOSE, RELEASETRANSFER, LIFTUP, FINISHTRANSFER, DROPWAIT, GETTHERE, COMEDOWN, PLACEDONE, LETGO, ARMUP, WAITARM,  PRELIMTRANSFER, PLACEWAIT
    }
    public enum DriveStates{
        IDLE,STARTDRIVING, ALIGN, NOTIFY,BASKETDRIVING, BASKETALIGN, BASKETNOTIFY
    }

    public DriveStates driveState= DriveStates.IDLE;
    public TransferStates transferState = TransferStates.IDLE;
    public enum LiftStates{
        LIFT,
        SERVO_OUT,
        SERVO_TURN,
        RETURN_WRIST,
        RETRACT,

        LIFT_DOWN,
        LIFT_UP_RETURN,

        RETURN,
        WRIST_OUT,

        REDROP,

        PUSH,

        CLOSE_DROPPER,
        AUTO_TURN, SENSOR, DRONEUP, DRONESHOOT, DRONEDOWN, CLOSED, DRONELIFT, IDLE
    }
    public LiftStates liftState = LiftStates.IDLE;
    boolean pickup=true;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public void init(){

        robot.init(hardwareMap, false,telemetry);
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        robot.readi2c();//TODO:Only read when needed

        robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
        robot.controller.getController().updateCombinedLocalizer();

       // robot.webcam.cameraClose();
       // robot.frontcam.setPixelPipeline();
        buttonTable=setupCheckButtons();
        if (Data.getBlue()){
            robot.limelight.pipelineSwitch(1);
            isBlue=1;

        } else {
            robot.limelight.pipelineSwitch(0);
            isBlue=-1;
        }
        ledState= LEDStates.CHANGESIDE;
        ledstate();
      //  robot.ledStrip.setColors(robot.ledColor);
       // robot.wrist.closeDropper();
       // robot.purpleClose();
        robot.controller.getController().setIsAuto(false);
    }
    public void init_loop(){
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        robot.readi2c();//TODO:Only read when needed
    /*    robot.readLimelight();
        telemetry.addData("Blue", Data.getBlue() );

        telemetry.addData("Distance",calculateExtension());
        if (robot.limelightResult != null) {
            telemetry.addData("Pipeline", robot.limelightResult.getPipelineIndex());
            telemetry.addData("Ty", robot.limelightResult.getTy());
        }
        telemetry.addData("stored pose",Data.getPose());
            telemetry.addData("stored heading",Math.toDegrees(Data.getHeading()));
        telemetry.update();
*/
        telemetry.addData("Blue", Data.getBlue() );
        telemetry.addData("stored pose",Data.getPose());
        telemetry.addData("stored heading",Math.toDegrees(Data.getHeading()));
        telemetry.update();


    }
        public void start(){
            for (LynxModule module : robot.allHubs) {
                module.clearBulkCache();
            }
            robot.setPose(Data.getPose().x, Data.getPose().y, Data.getHeading());
            robot.intake.catchOpen();
       //     robot.readi2c();
            robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
            robot.controller.getController().updateCombinedLocalizer();;
            robot.controller.getController().initzeroPower(Math.toRadians(0));
            robot.controller.getController().setIsAuto(false);
            robot.arm.armPreBasket();
            robot.intake.wristTransfer();
            robot.lift.liftFullDown();
            liftTimer.startTime();
            dropTimer.startTime();
        }


        public void loop() {
            for (LynxModule module : robot.allHubs) {
                module.clearBulkCache();
            }

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentcurrentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previouscurrentGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            robot.readi2c();//TODO:Only read when needed
            if (robot.intake.rumble) {
                gamepad1.rumbleBlips(2);
                robot.intake.rumble=false;
            }

         //   robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
            robot.controller.getController().updateCombinedLocalizer();
       //     prox=robot.outtakeProx;
        //    if (!robot.ledStrip.getDeviceClient().isArmed()) {ledOn=false;} else {ledOn=true;}

            checkButtons();
           // prox=robot.getProx();
           lift();
       //     wrist();
            drive();
            claw();
            arm();
            intake();
           ledstate();
        //    dropperstate();

           //reset IMU from blue side
            if(currentGamepad1.right_stick_button&& !buttonTable.get("g1rsb")){
                buttonTable.put("g1rsb", true);
                robot.resetYawManual();
                robot.ultrasonicLocalizer.setBlue(true);
                isBlue = 1;
                ledState= LEDStates.CHANGESIDE;
            }
            //reset IMU from red side
            if(currentGamepad1.left_stick_button&& !buttonTable.get("g1lsb")){
                buttonTable.put("g1lsb", true);
                robot.resetYawManual();
                robot.ultrasonicLocalizer.setBlue(false);
                isBlue = -1;
                ledState= LEDStates.CHANGESIDE;
            }

            //reset lift encoders
            if(currentGamepad2.left_stick_button&& !buttonTable.get("g2lsb")){
                buttonTable.put("g2lsb", true);
                robot.lift.resetLift();
            }

        //    telemetry.addData("LiftLeft",robot.lift.liftLeft.getCurrentPosition());
        //    telemetry.addData("LiftRight",robot.lift.liftRight.getCurrentPosition());
            telemetry.addData("Angle", Math.toDegrees(robot.getOrientation()));
            telemetry.addData("ODO Angle",Math.toDegrees(robot.sparkODO.getPose().h));

           // ledstate();
            telemetry.update();
            robot.update();
           robot.datalogWrapperElectricity.datalogWrite(transferState.toString());
            //telemetry.addData("Angle", Math.toDegrees( robot.getOrientation()));
            //telemetry.addData("rf",(robot.controller.getController().frontRight.lamprey.getVoltage()*Math.toRadians(360)) / 3.3);
            //telemetry.addData("lf",(robot.controller.getController().frontLeft.lamprey.getVoltage()*Math.toRadians(360)) / 3.3);
            //telemetry.addData("rr",(robot.controller.getController().backRight.lamprey.getVoltage()*Math.toRadians(360)) / 3.3);
            //telemetry.addData("lr",(robot.controller.getController().backLeft.lamprey.getVoltage()*Math.toRadians(360)) / 3.3);
            //telemetry.addData("imu",robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            //telemetry.update();
        }

    //code that manages the drive machine
    public void drive(){
   //   robot.controller.getController().updateLocalizer(robot.getOrientation());
        //force driving straight with joystick TODO:change deadbands?
       // if(currentGamepad1.left_bumper) {deadBand=.9;} else {deadBand=.2;}

        if (currentGamepad1.touchpad){//&&transferState==TransferStates.DROPWAIT

            if (robot.lift.targetSpecimen()) {
                driveState = DriveStates.STARTDRIVING;
            } else {
                driveState = DriveStates.BASKETDRIVING;
            }
        }


        if (currentGamepad2.guide&&!previousGamepad2.guide){
            robot.controller.abort=true;
            robot.controller.getController().setIsAuto(false);
            gamepad1.rumbleBlips(4);
            driveState= DriveStates.NOTIFY;
        }
        //speed reduction
        if(currentGamepad1.right_bumper){ multiplier=.4;} else {multiplier=1;}
/*
        //strafe right
        if (currentGamepad1.dpad_right && !buttonTable.get("g1dr")){
            buttonTable.put("g1dr", true);
            robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getXPos(), robot.controller.getController().getYPos()), robot.getOrientation()+Math.toRadians(5));
        }

        //strafe left
        if (currentGamepad1.dpad_left && !buttonTable.get("g1dl")){
            buttonTable.put("g1dl", true);
            robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getXPos(), robot.controller.getController().getYPos()), robot.getOrientation()-Math.toRadians(5));
        }

 */

        //get the x value, setting to 0 if within the deadband
        if (Math.abs(currentGamepad1.left_stick_x)<deadBand){
            deadx=0;
        }
        else {
            deadx=Math.signum(currentGamepad1.left_stick_x)*Range.clip(Math.pow(currentGamepad1.left_stick_x,2),0.1,1) ;//added square function
        }

        /*if (currentGamepad1.dpad_down && !buttonTable.get("g1dd")){
            buttonTable.put("g1dpd", true);
        //    System.out.println("14423 in DPD");
            deady=.5;
        }
        else if (currentGamepad1.dpad_up && !buttonTable.get("g1du")){
            buttonTable.put("g1dpu", true);
      //      System.out.println("14423 in DPU");
            deady=-.5;
        } else*/

        //get the y value, setting to 0 if within the deadband //TODO: proportional?
        if (Math.abs(currentGamepad1.left_stick_y)<deadBand){
            deady=0;
        }  else {

            deady= Math.signum(currentGamepad1.left_stick_y) *Range.clip(Math.pow(currentGamepad1.left_stick_y,2),0.1,1);//added square function
        }
       // System.out.println("14423 deady " + deady);

        //set our initial vector based on joystick and speed multiplier, correcting for red/blue
        Vector2D gamepadVector = new Vector2D( -deadx*isBlue*multiplier , -deady * isBlue*multiplier);
        Vector2D linearVelocityVector;
        //turn the initial vector based on our current orientation for field centric
        if (fieldCentric) { //TODO: option to turn off field centric?
            linearVelocityVector = gamepadVector.rotate(robot.getOrientation());
        } else {
            linearVelocityVector = gamepadVector;
        }
        //System.out.println("14423 linear  " + linearVelocityVector);
        //tell the swerve controller to drive to the vector
        robot.controller.getController().setLinearVelocityVector(linearVelocityVector.x, linearVelocityVector.y);

        //check for pivot deadband
        if (Math.abs(currentGamepad1.right_stick_x)<.2){
            deadrot=0;
        } else {deadrot=currentGamepad1.right_stick_x;
        }

        //tell robot controller to rotate based on joystick pivot TODO:set multiplier for speed
        robot.controller.getController().setAngularVelocity(- (deadrot/2)*(multiplier+.3) );
        //.1
        //if the joystick is on, then tell the controller to drive, otherwise stop
        if(Math.abs(currentGamepad1.left_stick_x) > 0.1 || Math.abs(currentGamepad1.left_stick_y) >0.1|| Math.abs(currentGamepad1.right_stick_x) > 0.1 ) {
            angle = linearVelocityVector.angle();
            robot.controller.getController().setIsAuto(false);
            robot.controller.getController().update();
        }
        else{
            if (!robot.controller.isDriving()) {
                robot.controller.getController().initzeroPower(angle);
            }

        }
    }

    public void intake(){


/*
            switch (ledState) {
                case EMPTY:
                    robot.intake.isIntakeSensor=false;
                    ledTimer.reset();
                    ledState= LEDStates.ONE;
                    currentGamepad1.rumbleBlips(1);
                    break;
                case ONE:
                    if (ledTimer.milliseconds()>500) {
                    robot.intake.isIntakeSensor=false;
                    currentGamepad1.rumbleBlips(2);
                    ledTimer.reset();
                    ledState= LEDStates.TWO;}
                    break;
                // robot.intake.intakeOff();
                // robot.intake.fourBarDrive();
                //  robot.wrist.closeDropper();
                //  openState=OpenStates.FULL;
                case TWO://TODO will need to debug for bounceback
                    if (ledTimer.milliseconds()>500) {
                        robot.intake.isIntakeSensor = false;
                        // robot.intake.intakeOff();
                        // robot.intake.fourBarDrive();
                        //    robot.wrist.closeDropper();
                        currentGamepad1.rumbleBlips(2);
                        ledState = LEDStates.CHECK;
                    }
                    break;


            }

    }
 */

        if(currentGamepad1.left_trigger > .75){
            robot.intake.extendoInFast();
            //robot.intake.extendAnalog(-currentGamepad1.left_trigger);
        } else if (currentGamepad1.left_trigger > 0){
            robot.intake.extendoInSlow();
            //robot.intake.extendAnalog(-currentGamepad1.left_trigger);
        }
        if(currentGamepad1.right_trigger > .75){
            robot.intake.extendoOutFast();
            //robot.intake.extendAnalog(currentGamepad1.right_trigger);
        } else if(currentGamepad1.right_trigger > 0){
            robot.intake.extendoOutSlow();
            //robot.intake.extendAnalog(currentGamepad1.right_trigger);
        }
       /* if(gamepad2.dpad_right && !buttonTable.get("g2dr")){
            buttonTable.put("g2dr", true);
            robot.intake.extendoOut();
        }
        if(gamepad2.dpad_left && !buttonTable.get("g2dl")){
            buttonTable.put("g2dl", true);
            robot.intake.extendoIn();
        }

        */

        if (currentGamepad1.guide && !previousGamepad1.guide) {
            robot.intake.intakeState= Intake.IntakeStates.OFF;

            //robot.lift.liftTransfer();
            robot.intake.wristHorizontal();
            robot.intake.extendTransfer();
            robot.intake.intakeStopperOpen();
        }
        //extend out and go horizontal TODO: Automate with webcam and color sensor
        if(currentGamepad1.a && !buttonTable.get("g1a")){
            buttonTable.put("g1a", true);
            //extend and turn wrist to horizontal, but do not turn on yet
            if(!robot.intake.isIntakeOn&&(robot.intake.intakeState== Intake.IntakeStates.OFF)){
                //TODO move into function for easy reproducing
                extend=calculateExtension();
                if (robot.lift.targetSpecimen()) {
                    robot.arm.armPickup();
                } else {
                   // robot.arm.armPlaceDone();
                    robot.arm.armTransfer();
                    robot.arm.clawWristFront();
                }

                ledState= LEDStates.EMPTY;
                //TODO Convert inches into levels
                robot.intake.extendFixed(extend); //TODO set by camera
                robot.intake.wristHorizontal(); //wristHorizontal();//Juwin
               // dropTimer.reset();
            }
            //turn off and come in
            if(robot.intake.intakeState== Intake.IntakeStates.ACQUIRED){//TODO add getting?
                robot.intake.intakeState= Intake.IntakeStates.OFF;//TODO needed?
                ledState = LEDStates.INTAKE;
               // robot.lift.liftTransfer();
                robot.intake.wristAuto();
                robot.intake.extendTransfer();
               openTimer.reset();
               robot.lift.setLiftPos(0);
                transferState = TransferStates.PRELIMTRANSFER;
               // dropTimer.reset();
            }
            //turn off/go horizontal, and await instructions
            if(robot.intake.intakeState== Intake.IntakeStates.ON||robot.intake.intakeState== Intake.IntakeStates.ONAUTO||robot.intake.intakeState== Intake.IntakeStates.ONBASKET||robot.intake.intakeState== Intake.IntakeStates.CLEAR){
                robot.intake.intakeState= Intake.IntakeStates.IDLE;
                robot.intake.wristHorizontal();
               // dropTimer.reset();
            }
            if(robot.intake.intakeState== Intake.IntakeStates.SPIT){
                robot.intake.intakeState= Intake.IntakeStates.ON;
               // robot.intake.wristHorizontal();
                //dropTimer.reset();
            }
            //come in
            if(robot.intake.intakeState== Intake.IntakeStates.IDLE){
                robot.intake.intakeState= Intake.IntakeStates.OFF;
                robot.intake.wristTransfer();
                robot.intake.extendoIn();
               // dropTimer.reset();
            }
        }

        //Turn the wrist down and the intake on
        if(currentGamepad1.x && !buttonTable.get("g1x")) {
            buttonTable.put("g1x", true);
            robot.intake.getIntakeProx();

            if  (robot.intake.intakeState== Intake.IntakeStates.CLEAR||robot.intake.intakeState== Intake.IntakeStates.SPIT){
               if (robot.lift.targetSpecimen()){
            robot.intake.intakeState= Intake.IntakeStates.ON;
                   robot.intake.intakeStopperClosed();} else {
                    robot.intake.intakeState= Intake.IntakeStates.ONBASKET;
                    robot.intake.intakeStopperClosed();
                }
            robot.intake.wristPickupTeleop();

           // robot.arm.clawWristIntake();
            robot.arm.openClaw(); } else {
                robot.intake.wristClear();
                robot.intake.intakeState= Intake.IntakeStates.CLEAR;
                robot.intake.intakeStopperClosed();
            }

        }
/*
        if(currentGamepad2.y && !buttonTable.get("g2y")){
            buttonTable.put("g2y", true);
            robot.intake.intakeState= Intake.IntakeStates.SPIT;
        }


 */
        if(currentGamepad1.b&& !buttonTable.get("g1b")){
            buttonTable.put("g1b", true);

          /*  if  (robot.intake.intakeState== Intake.IntakeStates.SPITBACK){
                robot.intake.intakeState= Intake.IntakeStates.OFF;
                robot.intake.wristHorizontal();

                // robot.arm.clawWristIntake();
                robot.arm.openClaw(); }
            else */
             if  (robot.intake.intakeState== Intake.IntakeStates.SPIT) {
                 robot.intake.intakeState= Intake.IntakeStates.OFF;
               // robot.intake.wristAuto();
               // robot.intake.intakeState = Intake.IntakeStates.SPITBACK;
            }

            else {
                robot.intake.wristSpit();
                robot.intake.intakeState = Intake.IntakeStates.SPIT;
            }
        }

        /*
            if (currentGamepad1.dpad_up && !buttonTable.get("g1du")){
            buttonTable.put("g1du", true);
            robot.intake.fourBarRaise();
        }

        if(currentGamepad2.dpad_right && !buttonTable.get("g2dr")){
            buttonTable.put("g2dr", true);
            robot.intake.extendoOut();
        }
        if(currentGamepad2.dpad_left && !buttonTable.get("g2dl")){
            buttonTable.put("g2dl", true);
            robot.intake.extendIn();
        }



         */
       /* if(currentGamepad1.left_bumper&& !buttonTable.get("g1lb")){
            buttonTable.put("g1lb", true);
            robot.intake.fourBarIn();
            robot.intake.intakeOff();
            robot.wrist.closeDropper();
            //robot.lift.liftLevel(1);
            dropTimer.reset();
        }*/
    }


    public void claw(){
       // if(currentGamepad2.right_trigger > 0.5 && currentGamepad2.left_trigger > 0.5){
         //   robot.wrist.turnerReturn();
          //  robot.lift.extendFull();
      //  }
       // else

        //strafe right
        if (currentGamepad1.dpad_right && !buttonTable.get("g1dr")){
            buttonTable.put("g1dr", true);
            robot.arm.closeClaw();
        }

        //strafe left
        if (currentGamepad1.dpad_left && !buttonTable.get("g1dl")){
            buttonTable.put("g1dl", true);
            robot.arm.openClaw();
        }
        /*
        if(currentGamepad2.right_trigger > 0.5){
           robot.arm.closeClaw();
         //   robot.lift.extendFull();
        }
        else if(currentGamepad2.left_trigger > 0.5){
            robot.arm.openClaw();

        }


         */



        /*if(currentGamepad2.right_bumper && !buttonTable.get("g2rb")){
            buttonTable.put("g2rb", true);
            //robot.wrist.rotateRight();
            robot.intake.setFourBarPos(robot.intake.intakeBar.getCurrentPosition()+15);
        }
        if(currentGamepad2.left_bumper && !buttonTable.get("g2lb")){
            buttonTable.put("g2lb", true);
           // robot.wrist.rotateLeft();
            robot.intake.setFourBarPos(robot.intake.intakeBar.getCurrentPosition()-15);
        } */

        if(currentGamepad2.a && !buttonTable.get("g2a")){
            buttonTable.put("g2a", true);
            if (transferState == TransferStates.PLACEWAIT||transferState == TransferStates.GETTHERE) {
                if (currentGamepad2.left_trigger > 0.5){
                    autodrive=true;
                } else {
                    autodrive=false;
                }
                transferState = TransferStates.FINISHPLACE;
                ledState= LEDStates.PLACE;}
            else
            {
                transferState = TransferStates.STARTPLACE;
                //decide whether to do something else on 2.a
                 }
        }
/*
        if(currentGamepad2.b && !buttonTable.get("g2b")) {
            buttonTable.put("g2b", true);
            //manually transfer.
            // TODO add start transfer when touch sensor activated
            //TODO break up transfer for more manual control
            transferState = TransferStates.STARTTRANSFER;

        }


 */
        if(currentGamepad1.left_bumper && !buttonTable.get("g1lb")) {
            buttonTable.put("g1lb", true);
            //manually transfer.
            // TODO add start transfer when touch sensor activated
            //TODO break up transfer for more manual control
            transferState = TransferStates.STARTTRANSFER;

        }


        if(currentGamepad1.y && !buttonTable.get("g1y")){
            buttonTable.put("g1y", true);
            if (transferState== TransferStates.DROPWAIT){
                transferState = TransferStates.STARTDROPOFF;
            } else {
            transferState= TransferStates.STARTPICKUP; }
            //TODO assign 1.y
        }
           /* OLD RESET CODE
           if (currentGamepad2.right_trigger>.5) {
                transferState = TransferStates.OPEN;
                openTimer.reset();
            } else



            if (transferState == TransferStates.PLACEWAIT) {
                transferState = TransferStates.PICKUP;
                openTimer.reset();
            } else
            if (transferState == TransferStates.DONE || transferState == TransferStates.FINISHPLACE){
                transferState = TransferStates.OPEN;
                openTimer.reset();
            }
            dropTimer.reset();

            */

    }
    public void arm(){


if (Math.abs(gamepad2.right_stick_x)>0) {
    if (gamepad2.right_stick_x < 0.9) {
        robot.arm.armRear();
        //   robot.lift.extendFull();
    } else if (gamepad2.right_stick_x > 0.9) {
        robot.arm.armFront();

    }
}
/*
        if (Math.abs(gamepad2.right_stick_y)>0) {
            if (gamepad2.right_stick_y < 0.9) {
                robot.arm.clawWristIn();
                //   robot.lift.extendFull();
            } else if (gamepad2.right_stick_y > 0.9) {
                robot.arm.clawWristOut();

            }
        }

 */

        switch (driveState){
            case IDLE:

                break;

            case STARTDRIVING://TODO: Add for RED
                robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.FORCEULTRA);

                if (isBlue == 1) {
                  //  if (Math.abs(robot.getOrientation())-Math.toRadians(270)>5){
                    robot.controller.turnOnly(Math.toRadians(270));
                //}
                } else {
                  //  if (Math.abs(robot.getOrientation())-Math.toRadians(90)>5) {
                        robot.controller.turnOnly(Math.toRadians(90));
                   // }
                }

                //    robot.controller.moveToPointPrecise(new Vector2D(-34.88, -62.75), Math.toRadians(270));
                robot.controller.update(robot.getOrientation());
                driveState = DriveStates.ALIGN;
                //   transferState = TransferStates.NOTIFY;


                break;
            case ALIGN:

                if (!robot.controller.isDriving()) {
                    if (isBlue == 1) {
                        //    double[][] points = {{-35.25, -61}, {-34, -47.5}, {-8, -47.5}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                        //   robot.controller.setCurvePrecise(points, Math.toRadians(270), 30);
                        robot.controller.moveToPointPrecise(new Vector2D(-36.25, -60.25 ), Math.toRadians(270));//-34.5,61
                    } else {
                    //        double[][] points = {{35.25, 60.75}, {34, 47.5}, {8, 47.5}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                      //     robot.controller.setCurvePrecise(points, Math.toRadians(270), 30);
                         robot.controller.moveToPointPrecise(new Vector2D(36.25, 60.25), Math.toRadians(90));//34.5//62.75
                    }

                    driveState = DriveStates.NOTIFY;
                } else {
                    robot.controller.update(robot.getOrientation());
                }
                break;
            case NOTIFY:
                if (!robot.controller.isDriving()) {

                    ledState = LEDStates.PICKUP;
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    gamepad1.rumbleBlips(3);
                    driveState= DriveStates.IDLE;
                    transferState= TransferStates.DROPWAIT;
                } else {
                    robot.controller.update(robot.getOrientation());
                }

                break;
            case BASKETDRIVING://TODO: Add for RED
                robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.BASKETULTRA);
                if (isBlue == 1) {
                    robot.controller.turnOnly(Math.toRadians(45));
                } else {
                    robot.controller.turnOnly(Math.toRadians(225));
                }

                //    robot.controller.moveToPointPrecise(new Vector2D(-34.88, -62.75), Math.toRadians(270));
                robot.controller.update(robot.getOrientation());
                driveState = DriveStates.BASKETALIGN;
                //   transferState = TransferStates.NOTIFY;


                break;
            case BASKETALIGN:

                if (!robot.controller.isDriving()) {
                    if (isBlue == 1) {
                        robot.controller.moveToPointPrecise(new Vector2D(58, -56), Math.toRadians(45));//-34.5
                    } else {
                        robot.controller.moveToPointPrecise(new Vector2D(-58, 56), Math.toRadians(225));//34.5//62.75
                    }

                    driveState = DriveStates.BASKETNOTIFY;
                } else {
                    robot.controller.update(robot.getOrientation());
                }
                break;
            case BASKETNOTIFY:
                if (!robot.controller.isDriving()) {

                    ledState = LEDStates.PICKUP;
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    gamepad1.rumbleBlips(3);
                    driveState= DriveStates.IDLE;
                    // transferState=TransferStates.DROPWAIT;
                } else {
                    robot.controller.update(robot.getOrientation());
                }

                break;
        }
        switch (transferState) {
            case IDLE:

                break;

            case STARTPICKUP: //move arm to pickup position
                robot.intake.wristAuto();
                robot.intake.extendTransfer();
                robot.lift.setLiftPos(0);
                robot.arm.clawWristFront();
                robot.arm.armPickup();
                openTimer.reset();
             //   robot.arm.clawWristPickup();
                transferState = TransferStates.PICKUP;
               // transferState = TransferStates.DROPWAIT;
                break;
            case PICKUP://if we are in pickup position, stop, or open claw if dropping off
                robot.intake.getExtendProx();
                   /* if (pickup){ //if picking up, get into position and open claw TODO set pickup in right spot

                        robot.arm.openClaw();
                        pickup=false;//reset for the next time
                        transferState = TransferStates.IDLE;
                    } else //if dropping off, get into drop of position, but wait for instruction to open claw
                    {
                       // robot.arm.clawWristDump();//TODO make sure can drop from here
                        pickup=true; //reset for the next time
                        openTimer.reset(); //reset time to give time for servo to move
                        transferState = TransferStates.DROPWAIT;
                    }

                    */
                if (robot.intake.isExtendTouch()||openTimer.milliseconds()> 800){
                    telemetry.addData("in place prox",robot.intake.extendProx);
                    telemetry.addData("in place ms",openTimer.milliseconds());
                    transferState = TransferStates.DROPWAIT;
                    openTimer.reset();
                  //  robot.intake.extendRelease();
                }


                break;
            case DROPWAIT:
                break;
            case STARTDROPOFF:
                robot.lift.setLiftPos(250);
                robot.arm.armTransferGetReady();
                /*
                //open the claw to drop
                if(openTimer.milliseconds() > 100){
                   robot.arm.openClaw();
                    openTimer.reset();
                    transferState = TransferStates.DROPOFF;
                }

                 */
                transferState = TransferStates.IDLE;
                break;
            case DROPOFF://wait for the block to drop
                if(openTimer.milliseconds() > 100){ //TODO tune time
                    pickup=true;
                     transferState = TransferStates.STARTPICKUP;
                }
                break;
            case OPEN: //open claw
               // if (openTimer.milliseconds()>100){
                robot.arm.openClaw();
                openTimer.reset(); //reset time to give time for servo to move
                transferState = TransferStates.RESET;
        //}

                //liftStates = LiftStates.LIFT_UP_RETURN;
               // openState=OpenStates.DONE;
                break;
            case RESET: //move the wrist back to pickup position
                if (openTimer.milliseconds()>100){
                 //   robot.arm.clawWristPickup();
                    transferState = TransferStates.IDLE;
                }
                break;
            case CLOSE:
                robot.arm.closeClaw();
                transferState = TransferStates.IDLE;
                break;

            case PRELIMTRANSFER:
                robot.intake.getExtendProx();
                if (robot.lift.targetSpecimen()) {
                    transferState = TransferStates.IDLE;
                } else
                {
                    if (robot.intake.isExtendTouch()||openTimer.milliseconds()> 3000){
                        transferState = TransferStates.STARTTRANSFER;
                        robot.intake.wristTransfer();
                       // robot.intake.extendRelease();
                     }
                }
                break;
            case STARTTRANSFER://move arm to transfer position
                robot.arm.armTransfer();
                robot.intake.wristTransfer();
               // robot.lift.setLiftPos(0 );
                robot.lift.liftFullDown();
               // robot.lift.setLiftPos(250);
              //  robot.arm.clawWristIntake();
               // robot.arm.armTransfer();


                robot.arm.openClaw();
                openTimer.reset();
                transferState = TransferStates.WAITARM;
                break;
            case WAITARM:
                if (openTimer.milliseconds()>50){
                   // robot.lift.liftFullDown();

                   // robot.arm.transferClaw();
                    transferState = TransferStates.ALIGNTRANSFER;
                }
                break;
            case ALIGNTRANSFER: //if we are in transfer position, rotate the wrist to grab the block
                if (robot.lift.isAtPosition()&&(openTimer.milliseconds()>25)) {//400
                 //   robot.arm.clawWristTransfer();
                    robot.intake.intakeState= Intake.IntakeStates.TRANSFER;
                    if (!robot.intake.isStopperSensor()) {//if (robot.intake.isIntakeSensor())
                        robot.intake.intakeStopperClosed();
                        robot.intake.intakeTransfer();
                    }
                    openTimer.reset(); //reset time to give time for servo to move
                    transferState = TransferStates.TRANSFER;
                }
                break;
            case TRANSFER://close the claw
                //TODO test the timing
             //   if ((!robot.intake.isIntakeSensor()&&dropTimer.milliseconds()>100)||dropTimer.milliseconds()>300){
              //  robot.intake.floorOpen();
              //  robot.intake.intakeTransfer();
                int timer=100;
                if (currentGamepad1.left_bumper){
                    timer=250;
                }
                if ((robot.intake.isStopperSensor() && openTimer.milliseconds()> timer ||openTimer.milliseconds()>350)) {//!robot.intake.isIntakeSensor()&&openTimer.milliseconds()>175)
                //    if (openTimer.milliseconds()>50){
                     /*   if (robot.lift.targetSpecimen()) {
                    robot.arm.closeClaw(); } else {
                            robot.arm.closeClawBasket();
                        }*/
                    robot.intake.intakeStopperOpen();
                    robot.intake.intakeState= Intake.IntakeStates.OFF;
                     robot.arm.closeClaw();

                    openTimer.reset();

                    transferState = TransferStates.RELEASETRANSFER;
                } /*else
                {robot.intake.intakeTransfer();} //TODO: Change this to move to position
                */
                break;
            case RELEASETRANSFER: //spit and raise arm
                if (openTimer.milliseconds()>250){
                    //robot.intake.intakeSpit();
                 //   robot.intake.intakeStopperClosed();
                    openTimer.reset();
                    gamepad2.rumbleBlips(2);
                    transferState = TransferStates.LIFTUP;}

                //liftStates = LiftStates.LIFT_UP_RETURN;
                // openState=OpenStates.DONE;
                break;
            case LIFTUP://lift up
                if (openTimer.milliseconds()>100) {//TODO: Set time
                    robot.intake.intakeOn();
                   // robot.intake.intakeState= Intake.IntakeStates.ON;
                    robot.lift.liftTransfer();
                   // robot.intake.intakeTransfer();

                    robot.intake.wristTransfer();
                    dropTimer.reset();
                    transferState = TransferStates.FINISHTRANSFER;
                }
                break;
            case FINISHTRANSFER://if up, then turn intake off, turn claw wrist out, and come down
             //   System.out.println("14423 in finish ");
                if (robot.lift.isAtPosition()) {
        //
          //      if (dropTimer.milliseconds()>500 ){
   // System.out.println("14423 in finish " +dropTimer.milliseconds());
               // robot.intake.intakeOff();
                robot.intake.intakeState= Intake.IntakeStates.OFF;
              //  robot.arm.clawWristIntake();
                pickup=false;


               // robot.lift.setLiftPos(0);
                transferState = TransferStates.IDLE;
            }
                break;


            case STARTPLACE: //lift to the right height and arm position

                    robot.lift.setLiftPos(robot.lift.liftLevel.getValue());
                    robot.lift.isPlacing = true;
                    if (robot.lift.targetSpecimen()) {
                        robot.arm.armPlace();//TODO ensure arm and wrist positions
                        openTimer.reset();
                        transferState = TransferStates.GETTHERE;
                    } else {
                        robot.arm.armPreBasket();
                        //robot.arm.armBasket();
                        openTimer.reset();
                    //    robot.arm.clawWristBasket();//TODO ensure arm and wrist positions
                        transferState = TransferStates.GETTHERE;
                    }

                break;
            case GETTHERE: //for now, just wait until at position, but could add other activity

                if (robot.lift.targetSpecimen()) {

                    if (robot.lift.isAtPosition()) {//&&robot.arm.isArmAtPlace()
                       robot.arm.closeStopper();

                        transferState = TransferStates.PLACEWAIT;
                    }
                } else {
                    if (openTimer.milliseconds()>150){
                        robot.arm.clawWristBack();
                    }
                    if (robot.lift.isAtPosition()) {//&&robot.arm.isArmAtBasket()

                        transferState = TransferStates.PLACEWAIT;
                    }
                }
                break;
            case PLACEWAIT:
                if (robot.lift.targetSpecimen()) {
                    if (openTimer.milliseconds()>250){
                   // robot.arm.clawWristBack();
                    robot.arm.closeClawSpecimen();

                    }
                }

                break;
            /*case DROPWAIT:

                break;

             */
            case FINISHPLACE:
                if (robot.lift.targetSpecimen()) {
                 //TODO add arm and claw movement to place the specimen!
                 //   robot.intake.extendLevels(10); //TODO set by camera
                 //   robot.intake.wristTransfer(); //wristHorizontal();Juwin
                 robot.lift.liftPlace();  //comment out for drive in
                    openTimer.reset();
                    transferState = TransferStates.LETGO;
                } else { //dropping in the basket
                  //  robot.arm.armBasketDrop();
                    robot.arm.armBackBasket();
                    openTimer.reset();
                    transferState = TransferStates.LETGO; //get back into transfer position
                    //add movement to new position
                    //add claw wrist
                    //add claw open
                }
                break;
            case LETGO:
                if (robot.lift.targetSpecimen()) {
                    if (robot.lift.isAtPosition()||openTimer.milliseconds()>550) {
                        robot.arm.openStopper();
                   //     robot.lift.liftPlace();//only here for ramming
                        robot.arm.openClaw();
                        openTimer.reset();
                        gamepad2.rumbleBlips(1);
                        transferState = TransferStates.ARMUP;
                    }
                } else {
                    if (openTimer.milliseconds() > 100) {
                        robot.arm.openClaw();
                     //   robot.arm.openStopper();
                        openTimer.reset();
                        transferState = TransferStates.ARMUP;
                    }
                }
                break;
            case ARMUP:
                if (robot.lift.targetSpecimen()) {
                    if (openTimer.milliseconds() > 250) {
                       // robot.arm.armPlaceDone(); //comment out for underside dropoff
                        if (autodrive) {
                            driveState = DriveStates.STARTDRIVING;
                        }
                        robot.arm.armPickup();
                        robot.arm.clawWristFront();
                        transferState = TransferStates.COMEDOWN;
                        openTimer.reset();
                    }
                } else {
                    if (openTimer.milliseconds() > 200) {
                      //  robot.arm.armBasket();

                        robot.arm.armTransfer();
                        robot.arm.clawWristFront();
                        // robot.arm.armPreBasket();
                        transferState = TransferStates.COMEDOWN;
                        openTimer.reset();
                    }
                }
                break;
            case COMEDOWN:
                if (robot.lift.targetSpecimen()) {
                    if (openTimer.milliseconds() >175) {
                        //TODO include test that place is done

                        robot.lift.setLiftPos(0);
                        //  robot.lift.liftFullDown();
                        ledState= LEDStates.PLACE;
                        transferState = TransferStates.STARTPICKUP;

                    }
                } else {
                if (openTimer.milliseconds() >175) {
                    //TODO include test that place is done

                   robot.lift.setLiftPos(0);
                  //  robot.lift.liftFullDown();

                    transferState = TransferStates.PLACEDONE;

                }
                }
                break;
/*
            case COMEDOWN:
                if (openTimer.milliseconds() >150) {
                    //TODO include test that place is done
                    robot.arm.armTransfer();
                    robot.lift.setLiftPos(0);
                    robot.arm.clawWristIntake();


                    transferState = TransferStates.PLACEDONE;
                }
                break;
  */
            case PLACEDONE:
                if(robot.lift.isAtPosition()){
                    transferState = TransferStates.IDLE;
                    ledState= LEDStates.PLACE;
                }
                break;

            /*
            case HALFDONE:
                if(openTimer.milliseconds()>250) {
                    robot.wrist.closeDropper();
                   // robot.lift.liftLevel(level+1);
                    robot.lift.returnBoard();
                    transferState = TransferStates.DONE;
                }
                break;

             */
        /*    case HALFRETRACT:
                if(openTimer.milliseconds()>350) {
                    robot.lift.liftLevel(level);
                    openState = OpenStates.DONE;
                }
                break;

         */
            case DONE:
                break;

        }
    }

    public void ledstate(){
      ledOn=true;//TODO choose different button to toggle LEDs
        /*  if (currentGamepad1.dpad_down && !buttonTable.get("g1dd")){
            buttonTable.put("g1dd", true);
            //    System.out.println("14423 in DPD");
            if (ledOn) {ledOn=false;} else {ledOn=true;}
        }*/



        if (ledOn) {
           if (ledState==oldledState){} else {
               oldledState=ledState;
                switch (ledState) {
                    case EMPTY:
                        robot.ledEmpty();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case PICKUP:
                        robot.ledPickup();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case INTAKE:
                        robot.ledIntake();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case PLACE:
                        robot.ledPlace();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case CHANGETARGET:
                        robot.ledTarget(isBlue==1);
                        robot.ledEmpty();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case CHANGESIDE:
                        robot.ledSide(isBlue==1);
                       // robot.ledTarget(isBlue==1);
                        robot.ledEmpty();
                        robot.ledStrip.setColors(robot.ledColor);
                        break;
                    case PIXEL:
                        robot.ledStrip.setColors(robot.pixel);
                        break;
                }
            }
        }


    }
    public void lift(){

        //lift up
        if(currentGamepad2.dpad_up && !buttonTable.get("g2du")){
            buttonTable.put("g2du", true);
            robot.lift.liftUp();
            liftTimer.reset();
            if(liftTimer.time() > 50){//TODO: Do we need the timer?

            }
        }

        //lift down
        if (currentGamepad2.dpad_down && !buttonTable.get("g2dd")){
            buttonTable.put("g2dd", true);
            robot.lift.liftDown();
            liftTimer.reset();
            if(liftTimer.time() > 50){//TODO: Do we need the timer?

            }
        }

       /* if(currentGamepad1.b&& !buttonTable.get("g1b")){
            buttonTable.put("g1b", true);
            dronehang=true;
            liftState = LiftStates.DRONELIFT;
        }

        */
        /*
        if(currentGamepad1.x){
           // robot.lift.liftPullUp();
            robot.lift.setIsHang(true);
            robot.lift.liftHangDown();
        }
        // else {robot.lift.setIsHang(false);}
         */
/*
        if (currentGamepad1.dpad_up && !buttonTable.get("g1du")) {
            buttonTable.put("g1du", true);
            robot.lift.setIsHang(true);

            robot.lift.liftHangUp();
        }
        if (currentGamepad1.dpad_down && !buttonTable.get("g1dd")) {
            buttonTable.put("g1dd", true);
            robot.lift.setIsHang(true);
            robot.lift.liftHangDown();
        }
*/
        if(currentGamepad2.y && !buttonTable.get("g2y")){
            buttonTable.put("g2y", true);
            if (robot.lift.liftLevel== Lift.LiftLevel.HANG) {
                robot.lift.setIsHang(true);
                robot.lift.liftHangPrep();
                robot.lift.setLiftPos(robot.lift.liftLevel.getValue()); //TODO: check postion
                robot.arm.armTransfer();
            }
        }

        if(currentGamepad2.x && !buttonTable.get("g2x")){
            buttonTable.put("g2x", true);
            robot.arm.openStopper();
            }




        if(currentGamepad2.b && !buttonTable.get("g2b")) {
            buttonTable.put("g2b", true);
            //manually transfer.
            if (robot.lift.liftLevel== Lift.LiftLevel.HANG) {
                robot.lift.setIsHang(true);
                robot.lift.liftHang();
            }

        }


        //TODO: change levels - basket? speciment?

        //cycle through specimen and baskets
        if(currentGamepad2.left_bumper && !buttonTable.get("g2lb")) {
            buttonTable.put("g2lb", true);
            robot.lift.liftLevel = robot.lift.liftLevel.previous();
            ledState = LEDStates.CHANGETARGET;
        }
           /* if (robot.lift.targetSpecimen()) {
                if (Data.getBlue()) {
                    robot.limelight.pipelineSwitch(1);

                } else {
                    robot.limelight.pipelineSwitch(0);
                }
            } else {
                robot.limelight.pipelineSwitch(2);
            }


        }*/
        if(currentGamepad2.right_bumper && !buttonTable.get("g2rb")) {
            buttonTable.put("g2rb", true);
            robot.lift.liftLevel = robot.lift.liftLevel.next();
            ledState = LEDStates.CHANGETARGET;
//        }
            if (robot.lift.targetSpecimen()) {
                if (isBlue==1) {
                    robot.limelight.pipelineSwitch(1);

                } else {
                    robot.limelight.pipelineSwitch(0);
                }
            } else {
                robot.limelight.pipelineSwitch(2);
            }

        }



/*
        switch(liftState){
            case LIFT:
                robot.lift.liftLevel(level);
                dropTimer.reset();
                liftState = LiftStates.SERVO_OUT;
                break;
            case SERVO_OUT:
                robot.lift.extendLevels(level);
                dropTimer.reset();
                liftState = LiftStates.WRIST_OUT;
                break;
            case WRIST_OUT:
                if(dropTimer.milliseconds() > 300){
                robot.wrist.wristOut();
                dropTimer.reset();
                openTimer.reset();

                //TODO add way to turn autoturn off incase orientation gets off
             //   liftStates = LiftStates.SENSOR;

               // dropTimer.reset();
                 liftState = LiftStates.AUTO_TURN;


            }
                break;

            case AUTO_TURN:
                robot.lift.extendTOBoard(prox);
                telemetry.addData("extendo position set",robot.lift.extendoPosition);
                telemetry.addData("extendo position actual",robot.lift.extendo.getPosition());
                //telemetry.addData("extendo position actual",robot.lift.extendo.getPosition());
                if(dropTimer.milliseconds() > 650) {
                    robot.wrist.turnerAuto(robot.getOrientation());
                    telemetry.addData("BoardSensorDistance", prox);
                    if (prox < 8.5) {//need to verify distance
                        ledState = LEDStates.BOARD;
                    } else {
                        ledState = LEDStates.CHECK;
                    }
                   // dropTimer.reset();
                    // liftStates = LiftStates.SENSOR;
                }
                break;
            case LIFT_UP_RETURN:
                if(dropTimer.milliseconds() > 400){
                    transferState = TransferStates.PLACEWAIT;
                    dropTimer.reset();
                    liftState = LiftStates.CLOSED;
                }
                break;
            case CLOSED:
                if(dropTimer.milliseconds() > 200){
                    robot.lift.liftLevel(level + 1);
                    robot.wrist.turnerReturn();
                    dropTimer.reset();
                    liftState = LiftStates.RETRACT;
                }
                break;
            case RETRACT:
                if(dropTimer.milliseconds() > 750){
                    robot.lift.extendMin();
                    dropTimer.reset();
                    liftState = LiftStates.RETURN_WRIST;
                }
                break;
            case RETURN_WRIST:
                if(dropTimer.milliseconds() > 100){
                    dropTimer.reset();
                    robot.wrist.wristIn();
                    liftState = LiftStates.LIFT_DOWN;
                }
                break;
            case LIFT_DOWN:
                if(dropTimer.milliseconds() > 400){
                    robot.lift.setLiftPos(0);
                    liftState = LiftStates.IDLE;
                }
            case IDLE:
                break;
            case SENSOR:
                if(dropTimer.milliseconds() > 200){
                    telemetry.addData("BoardSensorDistance", prox);
                    if (prox < 8.5) {//need to verify distance
                        ledState = LEDStates.BOARD;
                    } else {
                        ledState = LEDStates.CHECK;
                    }

                    robot.wrist.turnerAuto(robot.getOrientation());
                    dropTimer.reset();
                }
                break;
            case DRONELIFT:
                    if (dronehang) {
                        robot.lift.liftHang();
                    } else {
                        robot.lift.liftLevel(3);
                    }
                    dropTimer.reset();
                    liftState = LiftStates.DRONEUP;


                break;
            case DRONEUP:
        if (dropTimer.milliseconds()>500) {
            robot.wrist.droneUp();
            dropTimer.reset();
            liftState = LiftStates.DRONESHOOT;
        }
                break;
            case DRONESHOOT:
                if (dropTimer.milliseconds()>500) {
                    dropTimer.reset();
                    robot.wrist.droneShoot();
                    liftState = LiftStates.DRONEDOWN;
                }
                break;
            case DRONEDOWN:
                if (dropTimer.milliseconds()>300) {
                    robot.wrist.droneDown();
                    if (dronehang){

                        robot.lift.liftHang();
                        robot.wrist.wristLift();
                    }
                    else {
                       // robot.lift.liftLevel(0);
                        robot.lift.setLiftPos(0);
                    }
                    liftState = LiftStates.IDLE;
                }
                break;
        }

 */
        telemetry.addData("Level", robot.lift.liftLevel);
        telemetry.addData("lift",robot.lift.liftRear.getCurrentPosition());
        telemetry.addData("lift target",robot.lift.targetPosition);
        telemetry.addData("lift is down", robot.lift.isDown());
        telemetry.addData("Angle", Math.toDegrees(robot.getOrientation()));
        telemetry.addData("Extendo", robot.intake.extendoPosition);
        telemetry.addData("intakeSensor", robot.intake.isIntakeSensor());
        telemetry.addData("extendDistance",robot.intake.extendProx);
        telemetry.addData("intakeSensorDistance", robot.intake.intakeSensor.getState() );
        telemetry.addData("intakeSensorDistanceCached", robot.intake.intakeProx);

        telemetry.addData("armServoPosition", robot.arm.armServo.getPosition());
        telemetry.addData("block distance", extend);
        telemetry.addData("transfer State", transferState);
        telemetry.addData("localizer mode", robot.controller.getController().combinedLocalizer.poseMode);
        telemetry.addData("position",robot.controller.getController().getPositionVector());
        telemetry.addData("sparkposition",robot.controller.getController().combinedLocalizer.getSparkVector());
        telemetry.addData("ultraposition",robot.controller.getController().combinedLocalizer.getUltrasonicVector());
        telemetry.addData("hangposition",robot.lift.liftHang.getCurrentPosition());

    }

    public void checkButtons() {

        if (!currentGamepad2.a){buttonTable.put("g2a", false);}
        if (!currentGamepad2.start){buttonTable.put("g2start", false);}
        if (!currentGamepad1.a){buttonTable.put("g1a", false);}
        if (!currentGamepad1.y){buttonTable.put("g1y", false);}
        if (!currentGamepad2.y){buttonTable.put("g2y", false);}
        if (!currentGamepad2.b){buttonTable.put("g2b", false);}
        if (!currentGamepad2.x){buttonTable.put("g2x", false);}
        if (!currentGamepad1.b){buttonTable.put("g1b", false);}

        if (!currentGamepad1.x){buttonTable.put("g1x", false);}
        if (!currentGamepad1.dpad_up){buttonTable.put("g1du", false);}
        if (!currentGamepad1.dpad_down){buttonTable.put("g1dd", false);}
        if (!currentGamepad1.dpad_right){buttonTable.put("g1dr", false);}
        if (!currentGamepad1.dpad_left){buttonTable.put("g1dl", false);}
        if (!currentGamepad2.dpad_right){buttonTable.put("g2dr", false);}
        if (!currentGamepad2.dpad_down){buttonTable.put("g2dd", false);}
        if (!currentGamepad2.dpad_up){buttonTable.put("g2du", false);}
        if (!currentGamepad2.dpad_left){buttonTable.put("g2dl", false);}

        if (!currentGamepad2.left_stick_button){buttonTable.put("g2lsb",false);}
        if (!currentGamepad2.right_bumper){buttonTable.put("g2rb",false);}
        if (!currentGamepad2.left_bumper){buttonTable.put("g2lb", false);}
        if (!currentGamepad1.right_bumper){buttonTable.put("g1rb",false);}
        if (!currentGamepad1.left_bumper){buttonTable.put("g1lb", false);}
        if (!currentGamepad1.right_stick_button){buttonTable.put("g1rsb", false);}
        if (!currentGamepad1.left_stick_button){buttonTable.put("g1lsb", false);}



        for (Map.Entry<String, Boolean> entry : buttonTable.entrySet()) {
            String button = entry.getKey();
            Boolean isDown = entry.getValue();
            if (!isDown) {
                entry.setValue(false);
            }
        }
    }

    public HashMap<String, Boolean> setupCheckButtons(){
        HashMap<String, Boolean> lookupTable = new HashMap<>();
        lookupTable.put("g1a", false);
        lookupTable.put("g2a", false);
        lookupTable.put("g1x", false);
        lookupTable.put("g2x", false);
        lookupTable.put("g1b", false);
        lookupTable.put("g2b", false);
        lookupTable.put("g1y", false);
        lookupTable.put("g2y", false);
        lookupTable.put("g1dl", false);
        lookupTable.put("g2dl", false);
        lookupTable.put("g1dr", false);
        lookupTable.put("g2dr", false);
        lookupTable.put("g1du", false);
        lookupTable.put("g2du", false);
        lookupTable.put("g1dd", false);
        lookupTable.put("g2dd", false);
        lookupTable.put("g1rb", false);
        lookupTable.put("g2rb", false);
        lookupTable.put("g1lb", false);
        lookupTable.put("g2lb", false);
        lookupTable.put("g1lsb", false);
        lookupTable.put("g2lsb", false);
        lookupTable.put("g1rsb", false);
        lookupTable.put("g2rsb", false);
        lookupTable.put("g1start", false);
        lookupTable.put("g2start", false);


        return lookupTable;
    }

    private double calculateExtension(){
        if (robot.limelightResult != null) {
            double distance= 12.2 / Math.tan(Math.toRadians(17.8 + robot.limelightResult.getTy()))-7.5;//6.5
            //return (0.0263838*distance)+0.266513;//DSServo
            return (0.02097*distance)+0.266513;//AXON
        } else {
            return .5;
        }
    }

    public void stop() {
    //    robot.intake.extendo.setPwmDisable();
    //    robot.arm.armServo.setPwmDisable();
        robot.lift.setIsHang(true);
        robot.lift.liftHangDone();
        robot.update();
    }
}
