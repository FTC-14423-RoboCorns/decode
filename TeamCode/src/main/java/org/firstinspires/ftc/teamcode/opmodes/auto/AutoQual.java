package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.decode.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.decode.DecodeRobot;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Data;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.decode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.util.Vector2D;

/**
Primary auto opmode
 */
@Config
@Autonomous(group = "drive")
public class AutoQual extends OpMode {

    public DecodeRobot robot = new DecodeRobot(0, 0, 0);
    public boolean isRed = true;
    private boolean farstart = true;
    public int autoChoice = 1;
    private double angle = 0;
    private int delay = 0;

    private int shootCounter = 0;

    private int noMoveCycles = 0;
    double prox = 30;
    // private AprilTagPoseFtc ftcPose=null;
    //tnes are for testing in FTC Dashboard
    public static double X1;
    public static double Y1;
    public static double X2 = -6;
    public static double Y2 = -51;
    public static double X3 = -20;
    public static double Y3 = -50;
    public static double X4 = -36;
    public static double Y4 = -39;
    public static double ANGLE = 150;
    //set maxshots to a proper number
    public static int MAXSHOTS = 6;
    public static int MINSHOTS = 1;
    int cycleCount = 0;
    private boolean donePlace = false;
    private boolean notMoving = false;

    //These will be adjusted for our auto states
    private enum DriveState {
        SHOOT_1,
        PARK,
        TRAJECTORY_2,
        EXTRA,
        TURN,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,
        DROP,
        DONE_DROP,
        GETFROMSTACK,
        INTAKEOFF,
        PLACE2,
        PLACE3,
        STARTPLACE2,
        GETPOS,
        GET_TO_TAG,
        NEW_DONE_DROP,
        NO_PIXEL,
        PLACE_NOPIXEL,
        BACKOUT,
        APRIL,
        WAIT_1,
        PATH_3,
        GOTOBLOCKS,
        GETBLOCK,
        TURNBLOCK,
        PATH_1,
        PATH_2,
        EXTEND,
        PATH_4,PATH_4a,
        TURN2,
        EXTEND2,
        GETBLOCK2,
        RETRACT2,
        TURNBLOCK2,
        DROPBLOCK2,
        TURN3,
        GETBLOCK3,
        EXTEND3,
        RETRACT3,
        TURNBLOCK3,
        DROPBLOCK3,
        EXTENDBACK,
        INTAKE_2,
        STARTPICKUP2,
        IDLE,
        ALIGN,
        CLOSECLAW,
        STARTDRIVING,
        STARTLIFT2,
        STARTTRANSFER,
        ALIGNTRANSFER,
        TRANSFER,
        RELEASETRANSFER,
        LIFTUP,
        FINISHTRANSFER,
        EXTENDTIME,
        EXTENDIN,
        PARK,
        MOVEDROP,
        GETBLOCKSUB,
        LETGO2,
        ARMUP2,
        COMEDOWN2,
        ONWAIT2,
        STARTTRANSFER2,
        EXTENDIN2,
        ALIGNTRANSFER2,
        TRANSFER2,
        RELEASETRANSFER2,
        LIFTUP2,
        FINISHTRANSFER2,
        LETGO3,
        MOVEDROP3,
        ARMUP3,
        COMEDOWN3,
        ONWAIT3,
        STARTTRANSFER3,
        EXTENDIN3,
        ALIGNTRANSFER3,
        TRANSFER3,
        RELEASETRANSFER3,
        LIFTUP3,
        FINISHTRANSFER3,
        GOTOBLOCKS3,
        GETBLOCKSUB3,
        LETGO4,
        ARMUP4,
        MOVEDROP4,
        COMEDOWN4,
        GETBLOCK4,
        GETBLOCKSUB4,
        GOTOBLOCKS4,
        ONWAIT4,
        RETRACT4,
        STARTTRANSFER4,
        EXTENDIN4,
        ALIGNTRANSFER4,
        TRANSFER4,
        RELEASETRANSFER4,
        LIFTUP4,
        FINISHTRANSFER4,
        DROPBLOCK4,
        WAITARM2,
        WAITARM3,
        WAITARM4,
        EXTENDTIME2,
        PLACE, INTAKE_1,
    }

    //change enum values to match our choice of park locations
    private enum ParkState { //off the launch line - we can pick as many places we want
        CLOSE,
        FAR,
    }

    //we may not need - tracks the state of any given intake sequence
    private enum IntakeState {
        GETBALLS,
        ONWAIT,
        EXTEND,
        GETBLOCKSPECIMEN,
        ONWAITSPECIMEN,
        EXTENDSPECIMEN,
        GETBLOCKSUB,
        ONWAITSUB,
        EXTENDSUB,
        IDLE,
    }

    private IntakeState intakeState = IntakeState.IDLE;

    //we will need to change names and add more paths for our various ways of doing auto (which first, whether we dump, etc)
    private enum Path {
        ONETIME(2), //need to set values for number of shots for each path if that helps
        DUMP(5); //,

        // STOP;
        private final int defaultCycles;

        Path(final int newValue) {
            defaultCycles = newValue;
        }

        public int getCycles() {
            return defaultCycles;
        }

        private static Path[] vals = values();

        public Path next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public Path previous() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }

    private Path path = Path.ONETIME; //default - one set of two to start - we will add a dump as we get better

    //will need to change enums for clarity, but this will track our shooter operations. We may wind up moving into a shooter class
    private enum ShootState {
        DROP_1,
        DROP_2,
        DROP_3,
        PLACE_5,
        DROP_5,
        DROP_6,
        PLACE_1,

        SENSOR_DISTANCE,
        PLACE_2,
        PLACE_3,
        PLACE_4,
        INTAKE_OUT,
        SPIT,
        INTAKE_IN,
        DROP_WAIT,
        RETRACT,
        PLACE_AGAIN,
        LIFTOUT,
        DROP_RETRACT,
        TURNRETURN,
        SOMERETRACT,
        WRISTIN,
        DROP_NO_PIXEL,
        DROP_NO2,
        IDLE,
    }

    ParkState parkSide = parkSide = ParkState.FAR; //default for blue1;

    //this will track our spindexer states. We might wind up moving to a separate class
    public enum TransferStates {
        OPEN,
        PICKUP,
        DONE,
        FINISHPLACE,
        HALFDONE,
        STARTDROPOFF,
        DROPOFF,
        STARTPLACE,
        HALFRETRACT,
        IDLE,
        STARTPICKUP,
        STARTTRANSFER,
        TRANSFER,
        ALIGNTRANSFER,
        RESET,
        CLOSE,
        RELEASETRANSFER,
        LIFTUP,
        FINISHTRANSFER,
        DROPWAIT,
        GETTHERE,
        COMEDOWN,
        PLACEDONE,
        LETGO,
        PLACEWAIT,
    }

    private TransferStates transferState = TransferStates.IDLE;

    public class SwerveHeading {

        public Vector2D robotVector;
        public double robotHeading;

        public SwerveHeading(Vector2D v, double h) {
            robotVector = v;
            robotHeading = h;
        }
    }

    //if we want all headings stored in advance
    //private SwerveHeading[][][] driveArray = new SwerveHeading[2][2][6];
    //private SwerveHeading[][][] boardArray= new SwerveHeading[2][2][3];
    //private SwerveHeading[][][] boardArray2= new SwerveHeading[2][2][3];
    private int blueSide = 0;
    private int redSide = 1;
    private int cycles = 2; //set shots to a proper number
    private int color = blueSide;

    boolean isDPRDown = false;
    private DriveState driveState = DriveState.SHOOT_1;
    private ShootState dropState = ShootState.IDLE;
    public ElapsedTime dropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime extendTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public void init() {
        robot.init(hardwareMap, true, telemetry); //init the robot
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        //   robot.intake.extendo.setPwmEnable();
        //   robot.arm.armServo.setPwmEnable();

        //  robot.sparkODO.configureOtos();//only configure here, not in TeleOp

        robot.readi2c(); //TODO: Read only when needed
        //remember, Robot init cannot move anything, so all movement into place goes here
        //TODO: CLOSE GATE INIT
        //TODO: OPEN HOOD INIT
        //TODO: CENTER TURRET INIT
        robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
        cycles=path.getCycles();
        //include robot and subsytem initialization here

        dropTimer.startTime();
        robot.controller.getController().initzeroPower(Math.toRadians(270));

        robot.update();
    }

    public void init_loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        // robot.intake.catchOpen();
        // robot.intake.extendMin();
        // robot.intake.extendMin();
        // if (!gamepad1.dpad_right){isDPRDown=false;}
        robot.readi2c();
        //robot.intake.extendRelease();
        robot.controller.getController().combinedLocalizer.localizerUpdate();
        //TODO:VISION INIT TO SEE PATTERN
        //robot.controller.getController().updateLocalizer(robot.sparkODO.getPose().h);
        // robot.controller.getController().zeroPower(0);

        //set the path
        if (currentGamepad1.dpad_right & !previousGamepad1.dpad_right) {
            //  isDPRDown=true;
            path = path.next();
            cycles = path.getCycles();
        }
        //change number of shots
        if (currentGamepad1.dpad_up & !previousGamepad1.dpad_up) {
            //
            // isDPRDown = true;
            //manually control the number of cycles. But typically better to go with path default, depending on what we do
            cycles++;
            cycles = Range.clip(cycles, MINSHOTS, MAXSHOTS);
        }
        if (currentGamepad1.dpad_down & !previousGamepad1.dpad_down) {
            //  isDPRDown=true;
            cycles--;
            cycles = Range.clip(cycles, MINSHOTS, MAXSHOTS);
            //samples=4;
        }

        if (gamepad1.right_trigger > 0.5) {
            //TODO: ADD CODE TO OPEN AND CLOSE GATE FOR EASY BALL LOADING
        } else if (gamepad1.left_trigger > 0.5) {
            //robot.arm.openClaw();
        }
        //init the swerve, choose a side
        if (gamepad1.x) {
            isRed = true;

            color = redSide;
        } else if (gamepad1.b) {
            isRed = false;
            color = blueSide;
        }

        //set start position here
        if (gamepad1.a) {
            farstart = true; //may need an enum depending on how many start positions
        } else if (gamepad1.y) {
            farstart = false;
        }

        //record side for teleop
        Data.setRed(isRed);
        // if (gamepad1.dpad_right && !isDPRDown)

        telemetry.addData("Start Pos ay Far Start (true) Near Start (false)", farstart); //change to match start pos options
        telemetry.addData("isRed xb Red (true) Blue (false)", isRed);
        //   telemetry.addData("park trigger", parkSide);
        telemetry.addData("Auto Path choice dpad right", path);
        telemetry.addData("Cycle Count dpad up/down", cycles);
        //TODO:SEND TELEMETRY SHOWING PATTERN
        //add telemetry here for things we want to check - like apriltag distance and angle, making sure we can read obelisk, imu, etc.
        //  telemetry.addData("arm servo",robot.arm.armServo.getPosition());
        //  telemetry.addData("extendo",robot.intake.extendo.getPosition());
        // telemetry.addData("imu",(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        // telemetry.addData("localizer",Math.toDegrees((((robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)) + Math.toRadians(180)) % Math.toRadians(360)));

        //set our starting positions by path, starting pos, and color
        //TODO:SET STARTING POSITIONS
        if (path == Path.DUMP) {
            if (isRed) {
                //TODO: Set start poses
                robot.controller.getController().initzeroPower(Math.toRadians(0));
                if (farstart) {
                    robot.setPose(4, -64, Math.toRadians(270)); //-9
                } else {
                    robot.setPose(-23.5, -64, Math.toRadians(270));
                }
            } else {
                robot.controller.getController().initzeroPower(Math.toRadians(180));
                if (farstart) {
                    robot.setPose(-4, 64, Math.toRadians(90)); //9
                } else {
                    robot.setPose(23.5, 64, Math.toRadians(90));
                }
            }
        } else if (path == Path.ONETIME) {
            if (isRed) {
                //TODO: Set start poses
                robot.controller.getController().initzeroPower(Math.toRadians(0));
                robot.setPose(38.75, -64, Math.toRadians(180));
            } else {
                robot.controller.getController().initzeroPower(Math.toRadians(180));
                robot.setPose(-38.75, 64, Math.toRadians(0));
            }
        }

        // updatePoseUltrasonic();

        // robot.webcam.isBlue=isBlue;
        //robot checking telemetry
        telemetry.addData("Angle", Math.toDegrees(robot.getOrientation()));
        // telemetry.addData("imu",(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.addData("position", robot.controller.getController().getPositionVector());
        telemetry.addData("ODOAngle", Math.toDegrees(robot.sparkODO.getPose().h));
        telemetry.addData("AprilVector", robot.controller.getController().combinedLocalizer.getAprilTagVector());
        // telemetry.addData("localizer",Math.toDegrees((((robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)) + Math.toRadians(180)) % Math.toRadians(360)));
        //telemetry.addData("center",robot.webcam.center);
        //telemetry.addData("outtake prox",robot.outtakeProx);
        telemetry.update();
        //  robot.update();
        // robot.controller.update(robot.getOrientation());
    }

    public void start() {
        Data.setRed(isRed);
      //  robot.ultrasonicLocalizer.setBlue(isRed); //we likely won't use this this year
        robot.controller.getController().setIsAuto(true);
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        robot.readi2c(); //TODO: Read only what's needed
        robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
        robot.controller.getController().updateCombinedLocalizer();
        // robot.controller.getController().updateLocalizer(robot.sparkODO.getPose().h)
        // Start the state machines for the particular path
        //TODO: USE VISION TO READ PATTERN APRIL TAG AND STORE IT IN CASE WE EVER WANT TO DO ANYTHING WITH IT
        if (path == Path.DUMP) {
            dumpStart();
        } else if (path == Path.ONETIME) {
            onetimeStart();
        } else {}

        /*This was for testing auto
              double[][] points = {{50,50}, {45,10}, {32,4},{0,0}};

           robot.controller.setCurve(points, Math.toRadians(90),28);
         // robot.controller.moveToPoint(new Vector2D(0,48), Math.toRadians(0));


             */
        //if we use a webcam
        //  robot.webcam.setTagPipeline();
        // robot.webcam.cameraClose();
        robot.controller.update(robot.getOrientation());
        robot.update();
    }

    //note: loop only runs after the "start" code runs.
    public void loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        robot.readi2c(); //TODO read only if needed
        robot.controller.getController().updateCombinedLocalizer();
        //robot.controller.getController().updateLocalizer(robot.sparkODO.getPose().h);

        // prox=robot.outtakeProx;

        //  telemetry.addData("in2",autoChoice);
        //  telemetry.addData("pixel",placePixel);
        // telemetry.addData("extend prox: ", robot.intake.extendProx);
        telemetry.addData("claw rotate pos: ", robot.intake.extendProx);
        telemetry.addData("DriveState: ", driveState);
        telemetry.addData("DropState: ", dropState);
        telemetry.addData("XPos: ", robot.controller.getXPos());
        telemetry.addData("YPos: ", robot.controller.getYPos());
        //     telemetry.addData("IMUAngle", Math.toDegrees(robot.getOrientation()));
        telemetry.addData("Angle", Math.toDegrees(robot.getOrientation()));
        telemetry.addData("ODOAngle", Math.toDegrees(robot.sparkODO.getPose().h));
        telemetry.addData("ODOPoseX", robot.sparkODO.getPose().x);
        telemetry.addData("ODOPoseY", robot.sparkODO.getPose().y);
        telemetry.addData("position", robot.controller.getController().getPositionVector());
        //telemetry.addData("linearvelocityvector",robot.controller.getController().);

        // telemetry.addData("imu",(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        // telemetry.addData("localizer",Math.toDegrees((((robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)) + Math.toRadians(180)) % Math.toRadians(360)));

        telemetry.update();
        // telemetry.addData("imu",(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        // telemetry.addData("localizer",Math.toDegrees((((robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)) + Math.toRadians(180)) % Math.toRadians(360)));

        //this is the primary work in auto - it calls the intake state machine and the path macros.
        // Decode will likely do something similar. Our intake is much simpler and will likely not need an intake machine
        if (path == Path.DUMP) {
            //colors handled in main code
            intake();
            specimen();
        } else if (path == Path.ONETIME) {
            intake();
            onetime();
            //  System.out.println("14423 state: "+driveState);
        }
        //we leave this in in case we ever want to manually drive in auto
        //drive();

        if (!robot.controller.isDriving()) {
            robot.controller.zeroPower();
        }
        robot.controller.update(robot.getOrientation());
        robot.update();
        //save location for teleop
        Data.setPose(robot.controller.getController().combinedLocalizer.getSparkVector());
        Data.setHeading(robot.getOrientation());
        //  robot.datalogWrapper.datalogWrite();

        //         robot.datalogWrapper.datalogWrite();
    }

    public void stop() {
        robot.controller.getController().updateCombinedLocalizer();
        //   robot.intake.extendo.setPwmDisable();
        //   robot.arm.armServo.setPwmDisable();
        Data.setPose(robot.controller.getController().combinedLocalizer.getSparkVector());
        Data.setHeading(robot.getOrientation());
        //  robot.arm.armServo.setPwmDisable();
    }

    /*
     * Blue Auto
     */

    //for use with webcam
    /* public AprilTagPoseFtc getTag(int tagcheck){
            AprilTagPoseFtc tempTag;
            if (isNull(robot.webcam.tagPoseData[tagcheck])){
                tempTag=null;
            }
            else {
                tempTag=robot.webcam.tagPoseData[tagcheck];

            }
            return tempTag;
        }


        */
    //We likely won't need this since our intake is basically on and off, which can be handled by states (and sensors) in the intake subsystem
    //intothedeep needed more because intake required several subsystems and complex timing depending on location
    public void intake() {
        switch (intakeState) {
            case IDLE:
                break;
            case GETBLOCKSPECIMEN:
                if (intakeTimer.milliseconds() > 100) {
                    //150
                    robot.intake.intakeStopperClosed();
                    robot.intake.wristPickup();
                    robot.intake.intakeState = Intake.IntakeStates.ONAUTO;

                    intakeState = IntakeState.ONWAITSPECIMEN;
                    intakeTimer.reset();
                }

                break;
            case ONWAITSPECIMEN:
                robot.intake.getIntakeProx();
                if (intakeTimer.milliseconds() > 100) {
                    robot.intake.wristPickup();
                    intakeTimer.reset();
                    extendTimer.reset();
                    intakeState = IntakeState.EXTENDSPECIMEN;
                }

                break;
            case EXTENDSPECIMEN:
                //   System.out.println("14423 intakestate"+robot.intake.intakeState+"extendo Pos "+ robot.intake.extendo.getPosition());
                if (intakeTimer.milliseconds() > 25) {
                    //TODO add getting,separate extend, move off
                    if (
                        (robot.intake.intakeState == Intake.IntakeStates.ACQUIREDAUTO ||
                            robot.intake.intakeState == Intake.IntakeStates.GETTINGAUTO)
                    ) {
                        //||robot.intake.intakeState== Intake.IntakeStates.GETTINGBASKET
                        //   System.out.println("14423 in acquired intake state "+robot.intake.intakeState+" beam "+robot.intake.isIntakeSensor()+" stopper "+ robot.intake.isStopperSensor());
                        intakeTimer.reset();
                        dropTimer.reset();
                        // robot.intake.gripperClose();
                        intakeState = IntakeState.IDLE;
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        // driveState = DriveState.RETRACT;
                    } else if (
                        robot.intake.extendo.getPosition() >= robot.intake.MAXEXTEND && extendTimer.milliseconds() > 700
                    ) {
                        //    System.out.println("14423 in acquired intake state extendo"+robot.intake.extendo.getPosition());
                        dropTimer.reset();
                        intakeTimer.reset();
                        // robot.intake.gripperClose();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        intakeState = IntakeState.IDLE;
                        //driveState = DriveState.RETRACT;
                    } else {
                        if (cycleCount < 5) {
                            robot.intake.extendoOutFast();
                        }
                        //was fast
                        else {
                            //  if (dropTimer.milliseconds() > 500) {//500
                            robot.intake.extendoOutSlow();
                            // }
                        }
                        intakeTimer.reset();
                    }
                }
                break;
            case GETBALLS:
                if (cycleCount > 3) {
                    if (intakeTimer.milliseconds() > 400) {
                        robot.intake.intakeStopperClosed();
                        robot.intake.wristPickupSub();
                        robot.intake.intakeState = Intake.IntakeStates.ONBASKET;

                        intakeState = IntakeState.ONWAIT;
                        intakeTimer.reset();
                    }
                } else {
                    if (intakeTimer.milliseconds() > 100) {
                        //150
                        robot.intake.intakeStopperClosed();
                        robot.intake.wristPickup();
                        robot.intake.intakeState = Intake.IntakeStates.ONBASKET;

                        intakeState = IntakeState.ONWAIT;
                        intakeTimer.reset();
                    }
                }

                break;
            case ONWAIT:
                robot.intake.getIntakeProx();
                if (intakeTimer.milliseconds() > 100) {
                    if (cycleCount > 3) {
                        robot.intake.wristPickupSub();
                    } else {
                        robot.intake.wristPickup();
                    }
                    intakeTimer.reset();
                    extendTimer.reset();
                    intakeState = IntakeState.EXTEND;
                }

                break;
            case EXTEND:
                if (intakeTimer.milliseconds() > 25) {
                    //TODO add getting,separate extend, move off
                    if ((robot.intake.intakeState == Intake.IntakeStates.ACQUIRED)) {
                        //||robot.intake.intakeState== Intake.IntakeStates.GETTINGBASKET
                        intakeTimer.reset();
                        dropTimer.reset();
                        // robot.intake.gripperClose();
                        intakeState = IntakeState.IDLE;
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        // driveState = DriveState.RETRACT;
                    } else if (cycleCount > 3 && extendTimer.milliseconds() > 1000) {
                        dropTimer.reset();
                        intakeTimer.reset();
                        // robot.intake.gripperClose();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        intakeState = IntakeState.IDLE;
                    } else if (
                        robot.intake.extendo.getPosition() >= robot.intake.MAXEXTEND &&
                        extendTimer.milliseconds() > 1200
                    ) {
                        dropTimer.reset();
                        intakeTimer.reset();
                        // robot.intake.gripperClose();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        intakeState = IntakeState.IDLE;
                        //driveState = DriveState.RETRACT;
                    } else {
                        if (cycleCount < 4) {
                            robot.intake.extendoOutFast();
                        }
                        //was fast
                        else {
                            //  if (dropTimer.milliseconds() > 500) {//500
                            // robot.intake.extendoOutSlow();
                            // }
                        }
                        intakeTimer.reset();
                    }
                }
                break;
            case GETBLOCKSUB:
                if (intakeTimer.milliseconds() > 750) {
                    robot.intake.intakeStopperClosed();
                    robot.intake.wristPickupSub();
                    robot.intake.intakeState = Intake.IntakeStates.ONBASKET;

                    intakeState = IntakeState.ONWAITSUB;
                    intakeTimer.reset();
                }

                break;
            case ONWAITSUB:
                robot.intake.getIntakeProx();
                if (intakeTimer.milliseconds() > 100) {
                    robot.intake.wristPickupSub();

                    intakeTimer.reset();
                    extendTimer.reset();
                    intakeState = IntakeState.EXTENDSUB;
                }

                break;
            case EXTENDSUB:
                if (intakeTimer.milliseconds() > 25) {
                    //TODO add getting,separate extend, move off
                    if ((robot.intake.intakeState == Intake.IntakeStates.ACQUIRED)) {
                        //||robot.intake.intakeState== Intake.IntakeStates.GETTINGBASKET
                        intakeTimer.reset();
                        dropTimer.reset();
                        // robot.intake.gripperClose();
                        intakeState = IntakeState.IDLE;
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        // driveState = DriveState.RETRACT;
                    } else if (extendTimer.milliseconds() > 1400) {
                        dropTimer.reset();
                        intakeTimer.reset();
                        // robot.intake.gripperClose();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        intakeState = IntakeState.IDLE;
                    } else if (
                        robot.intake.extendo.getPosition() >= robot.intake.MAXEXTEND && extendTimer.milliseconds() > 400
                    ) {
                        dropTimer.reset();
                        intakeTimer.reset();
                        // robot.intake.gripperClose();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        intakeState = IntakeState.IDLE;
                        //driveState = DriveState.RETRACT;
                    } else {
                        robot.intake.extendoOutSlow();

                        intakeTimer.reset();
                    }
                }
                break;
        }
    }

    public void onetimeStart() {
        robot.limelight.pipelineSwitch(4);//TODO:SET VISION TO PROPER APRILTAG TARGET
        //TODO: START FLYWHEEL SPINUP TO SPEED BASED ON NEAR/FAR POSITION
        //TODO: TURN TURRET TO GOAL

        /*
        if (isRed) {
            robot.controller.moveToPoint(new Vector2D(58.5, -52.5), Math.toRadians(248)); //-53 ,248
        } else {
            robot.controller.moveToPoint(new Vector2D(-58.5, 52.5), Math.toRadians(68));
        }

         */
    }

    //this does most of the work for auto. Each state is very short, with the next state triggered by time or sensor.
    //we will need to figure out how to ensure we have three balls
    public void onetime() {
        //  robot.controller.update(robot.getOrientation());
        switch (driveState) {
            case SHOOT_1: //specimenCount=0

                if (robot.shooter.isAimed() && robot.shooter.isAtSpeed()) {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    robot.shooter.shootUntilEmpty();
                    driveState = DriveState.WAIT_1;
                }
                break;
            case WAIT_1:
                if (robot.intake.isEmpty()) {
                    //TODO:Add "isEmpty" to intake subsystem using sensors
                    cycleCount++;


                    if (cycleCount==1) {
                        driveState = DriveState.PATH_1;
                    }
                    else if (cycleCount ==2) {
                        driveState = DriveState.PATH_3;
                    } else if (cycleCount == 3) {
                        driveState = DriveState.PARK;
                    }
                }
                break;
            case PATH_1:
                //TODO: robot.controller.setPath to first set of balls
                if (isRed) {
                    //red path
                } else {
                    //blue path
                }
                driveState = DriveState.INTAKE_1;


                break;
            case INTAKE_1:
                if (robot.controller.remainingDistance < 6) {
                    intakeState = IntakeState.GETBALLS;
                    //TODO: Create Intake states that turns on intake and then turns off when 3 balls in place
                }

                if (!robot.controller.isDriving()) {
                    //TODO: robot.controller.setPath to shooting position
                    if (isRed) {
                        //red path
                    } else {
                        //blue path
                    }
                    driveState = DriveState.PATH_2;

                }
                break;

            case PATH_2:

                if (!robot.controller.isDriving()) {
                    //TODO: robot.controller.setPath to shooting position
                    if (isRed) {
                        //red path
                    } else {
                        //blue path
                    }
                    driveState = DriveState.SHOOT_1;
                }

                break;
            case PATH_3:
                //TODO: robot.controller.setPath to second set of balls
                if (isRed) {
                    //red path
                } else {
                    //blue path
                }
                driveState = DriveState.INTAKE_2;

                break;
            case INTAKE_2:
                if (robot.controller.remainingDistance < 6) {
                    intakeState = IntakeState.GETBALLS;
                    //TODO: Create Intake states that turns on intake and then turns off when 3 balls in place
                }

                if (!robot.controller.isDriving()) {
                    //TODO: robot.controller.setPath to shooting position
                    if (isRed) {
                        //red path
                    } else {
                        //blue path
                    }
                    driveState = DriveState.PATH_4;

                }
                break;
            case PATH_4:
                if (!robot.controller.isDriving()) {
                    //TODO: robot.controller.setPath to shooting position
                    if (isRed) {
                        //red path
                    } else {
                        //blue path
                    }
                    driveState = DriveState.SHOOT_1;
                }
                  break;
              case PARK:
                  //TODO: Set paths based on our various park states and blue or red side
                  switch (parkSide) {
                      case FAR:
                          if (isRed) {
                              //red path
                          } else {
                              //blue path
                          }
                          break;

                      case CLOSE:
                          if (isRed) {
                              //red path
                          } else {
                              //blue path
                          }
                          break;
                  }


                          ]

                  break;



            case STARTTRANSFER:
                if (dropTimer.milliseconds() > 100) {
                    //500
                    //    robot.controller.moveToPoint(new Vector2D(61, -58), Math.toRadians(225));
                    dropTimer.reset();
                    driveState = DriveState.EXTENDIN;
                }

                robot.lift.liftFullDown();
                // robot.lift.setLiftPos(0);
                // robot.arm.transferClaw();

                break;
            case EXTENDIN:
                robot.intake.getExtendProx();

                if (robot.intake.isExtendTouch() || dropTimer.milliseconds() > 1000) {
                    //800//1100
                    driveState = DriveState.ALIGNTRANSFER;
                    robot.intake.wristTransfer();
                }

                break;
            case ALIGNTRANSFER: //if we are in transfer position, rotate the wrist to grab the block
                if (robot.lift.isAtPosition() && (dropTimer.milliseconds() > 50)) {
                    //400
                    //   robot.arm.clawWristTransfer();
                    //reset time to give time for servo to move
                    robot.intake.intakeState = Intake.IntakeStates.TRANSFER;
                    if (!robot.intake.isStopperSensor()) {
                        //robot.intake.isIntakeSensor()
                        robot.intake.intakeStopperClosed();
                        robot.intake.intakeTransfer();
                    }
                    dropTimer.reset();
                    driveState = DriveState.TRANSFER;
                }

                break;
            case TRANSFER: //close the claw
                //TODO: test the timing
                // robot.intake.intakeStopperClosed();
                robot.intake.intakeTransfer();
                if (
                    (robot.intake.isStopperSensor() && dropTimer.milliseconds() > 135) || dropTimer.milliseconds() > 450
                ) {
                    //350//(!robot.intake.isIntakeSensor()&&dropTimer.milliseconds()>50)//reduced from 300 and 500 because we are already getting
                    //  System.out.println("14423 Sensor "+robot.intake.isIntakeSensor());
                    //  System.out.println("14423 Timer "+dropTimer.milliseconds());
                    //      if (dropTimer.milliseconds()>900){
                    robot.intake.intakeStopperOpen();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.RELEASETRANSFER; //TODO: Change this to move to position
                }
                break;
            case RELEASETRANSFER: //spit and raise arm
                if (dropTimer.milliseconds() > 150) {
                    //250
                    //robot.intake.intakeSpit();
                    dropTimer.reset();
                    //     robot.intake.intakeStopperClosed();
                    driveState = DriveState.LIFTUP;
                }

                //liftStates = LiftStates.LIFT_UP_RETURN;
                // openState=OpenStates.DONE;
                break;
            case LIFTUP: //lift up
                if (dropTimer.milliseconds() > 100) {
                    //TODO: Set time
                    robot.intake.intakeTransfer();
                    //  robot.lift.liftTransferAuto();
                    robot.intake.wristTransfer();
                    robot.lift.setLiftPos(Lift.LiftLevel.BASKETHIGH.getValue());

                    robot.lift.isPlacing = true;

                    dropTimer.reset();
                    driveState = DriveState.FINISHTRANSFER;
                }
                break;
            case FINISHTRANSFER: //if up, then turn intake off, turn claw wrist out, and come down
                //if (robot.lift.isAtPosition()) {
                if (dropTimer.milliseconds() > 400) {
                    //  robot.intake.intakeOff();
                    //  robot.intake.extendFull();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.arm.clawWristIntake();
                    //pickup=false;
                    // robot.lift.setLiftPos(0);
                    robot.arm.closeClawBasket();
                    robot.arm.armPreBasket();
                    robot.arm.clawWristOver();

                    /*  if (isBlue) {
                        robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(45));
                    } else {
                        robot.controller.moveToPoint(new Vector2D(-59,59), Math.toRadians(225));
                    }


                   */
                    driveState = DriveState.TRAJECTORY_2;
                }
                break;
            case TRAJECTORY_2: //specimenCount=1
                robot.arm.armPreBasket();
                // robot.arm.clawWristBack();
                robot.arm.clawWristOver();

                //  robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);

                // robot.arm.armPreBasket();
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(30))) {
                    robot.intake.extendMost();
                    robot.intake.wristPickup();
                    robot.intake.intakeState = Intake.IntakeStates.ONBASKET;
                }
                // robot.intake.extendLevels(24);
                //robot.arm.clawWristBack();
                // robot.arm.armBasketAuto();

                if (!robot.controller.isDriving() && robot.lift.isAtBasketPosition()) {
                    intakeState = IntakeState.GETBALLS;
                    driveState = DriveState.LETGO2;
                    dropTimer.reset();
                }
                break;
            case LETGO2:
                if (dropTimer.milliseconds() > 400) {
                    //220

                    cycleCount++; //specimenCOunt=2

                    robot.arm.armBackBasket();
                    //robot.arm.openClaw();
                    // robot.arm.openClaw();
                    dropTimer.reset();
                    driveState = DriveState.DROPBLOCK2;
                }
                break;
            case DROPBLOCK2:
                if (dropTimer.milliseconds() > 100) {
                    //350
                    robot.arm.openClaw();
                    //robot.arm.armPlaceDone();

                    driveState = DriveState.ARMUP2;
                }
                break;
            case ARMUP2:
                if (dropTimer.milliseconds() > 200) {
                    robot.arm.clawWristFront();
                    robot.arm.armTransfer();
                    driveState = DriveState.COMEDOWN2;
                    dropTimer.reset();
                }

                break;
            case COMEDOWN2:
                if (dropTimer.milliseconds() > 175) {
                    //TODO include test that place is done
                    robot.lift.setLiftPos(0); //250

                    driveState = DriveState.ONWAIT2;
                }

                break;
            case ONWAIT2:
                if (intakeState == IntakeState.IDLE) {
                    driveState = DriveState.RETRACT2;
                }
                break;
            case RETRACT2:
                if (dropTimer.milliseconds() > 100) {
                    //250

                    robot.intake.wristTransfer();

                    robot.arm.armTransfer();
                    robot.intake.extendTransfer();

                    driveState = DriveState.STARTTRANSFER2;
                    //driveState = DriveState.IDLE;
                    dropTimer.reset();
                }

                break;
            case STARTTRANSFER2:
                //Start moving while transferring

                //robot.intake.extendTransfer();

                if (dropTimer.milliseconds() > 100) {
                    //    robot.controller.moveToPoint(new Vector2D(61, -58), Math.toRadians(225));
                    dropTimer.reset();
                    driveState = DriveState.EXTENDIN2;
                }
                // double[][] points = {{59, -59}, {58, -58},{57, -57},{robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y}};
                // robot.controller.setCurve(points, Math.toRadians(225), 30);

                // robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(225));

                // robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(225));
                //    }

                robot.lift.liftFullDown();
                // robot.lift.setLiftPos(0);
                // robot.arm.transferClaw();

                break;
            case EXTENDIN2:
                robot.intake.getExtendProx();

                if (robot.intake.isExtendTouch() || dropTimer.milliseconds() > 1000) {
                    //800//1100
                    driveState = DriveState.ALIGNTRANSFER2;
                    robot.intake.wristTransfer();
                }

                break;
            case ALIGNTRANSFER2: //if we are in transfer position, rotate the wrist to grab the block
                if (robot.lift.isAtPosition() && (dropTimer.milliseconds() > 50)) {
                    //400
                    //   robot.arm.clawWristTransfer();
                    //reset time to give time for servo to move
                    robot.intake.intakeState = Intake.IntakeStates.TRANSFER;
                    if (!robot.intake.isStopperSensor()) {
                        //robot.intake.isIntakeSensor()
                        robot.intake.intakeStopperClosed();
                        robot.intake.intakeTransfer();
                    }
                    dropTimer.reset();
                    driveState = DriveState.TRANSFER2;
                }

                break;
            case TRANSFER2: //close the claw
                //TODO: test the timing
                // robot.intake.intakeStopperClosed();
                robot.intake.intakeTransfer();
                if (
                    (robot.intake.isStopperSensor() && dropTimer.milliseconds() > 135) || dropTimer.milliseconds() > 450
                ) {
                    //350//(!robot.intake.isIntakeSensor()&&dropTimer.milliseconds()>50)//reduced from 300 and 500 because we are already getting
                    //  System.out.println("14423 Sensor "+robot.intake.isIntakeSensor());
                    //  System.out.println("14423 Timer "+dropTimer.milliseconds());
                    //      if (dropTimer.milliseconds()>900){
                    robot.intake.intakeStopperOpen();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.RELEASETRANSFER2; //TODO: Change this to move to position
                }
                break;
            case RELEASETRANSFER2: //spit and raise arm
                if (dropTimer.milliseconds() > 150) {
                    //250
                    //robot.intake.intakeSpit();
                    dropTimer.reset();
                    //    robot.intake.intakeStopperClosed();
                    driveState = DriveState.LIFTUP2;
                }

                //liftStates = LiftStates.LIFT_UP_RETURN;
                // openState=OpenStates.DONE;
                break;
            case LIFTUP2: //lift up
                if (dropTimer.milliseconds() > 100) {
                    //TODO: Set time
                    robot.intake.intakeTransfer();
                    //    robot.lift.liftTransferAuto();
                    robot.lift.setLiftPos(Lift.LiftLevel.BASKETHIGH.getValue());
                    dropTimer.reset();
                    driveState = DriveState.FINISHTRANSFER2;
                }
                break;
            case FINISHTRANSFER2: //if up, then turn intake off, turn claw wrist out, and come down
                //  if (robot.lift.isAtPosition()) {
                if (dropTimer.milliseconds() > 250) {
                    //  robot.intake.intakeOff();
                    //  robot.intake.extendFull();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.arm.clawWristIntake();
                    //pickup=false;
                    // robot.lift.setLiftPos(0);

                    robot.lift.isPlacing = true;
                    robot.arm.closeClawBasket();
                    robot.arm.armPreBasket();
                    //  robot.arm.clawWristBack();
                    robot.intake.wristTransfer();

                    /*  if (isBlue) {
                        robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(45));
                    } else {
                        robot.controller.moveToPoint(new Vector2D(-59,59), Math.toRadians(225));
                    }


                   */
                    driveState = DriveState.TRAJECTORY_3;
                    // driveState = DriveState.IDLE;
                }
                break;
            case TRAJECTORY_3: //specimencCount=2
                robot.arm.armPreBasket();
                robot.arm.clawWristOver();

                //    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);

                // robot.arm.armPreBasket();
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(20))) {
                    //robot.intake.extendMost();
                    robot.intake.extendLevels(23);
                }
                // robot.intake.extendLevels(24);
                //robot.arm.clawWristBack();
                // robot.arm.armBasketAuto();

                if (!robot.controller.isDriving() && robot.lift.isAtBasketPosition()) {
                    driveState = DriveState.LETGO3;
                    dropTimer.reset();
                }
                break;
            case LETGO3:
                if (dropTimer.milliseconds() > 0) {
                    //220

                    cycleCount++; //specimenCount=3

                    // if (dropTimer.milliseconds() > 250) {
                    robot.arm.armBackBasket();
                    // robot.arm.openClaw();
                    dropTimer.reset();
                    driveState = DriveState.MOVEDROP3;
                    // }
                }

                break;
            case MOVEDROP3:
                if (dropTimer.milliseconds() > 25) {
                    if (isRed) {
                        robot.controller.moveToPoint(new Vector2D(56, -43), Math.toRadians(305));
                    } else {
                        // robot.controller.moveToPoint(new Vector2D(-57, 41), Math.toRadians(127));
                        robot.controller.moveToPoint(new Vector2D(-56, 43), Math.toRadians(125));
                    }
                    //  robot.arm.openClaw(); ///???add here for faster open?
                    dropTimer.reset();
                    driveState = DriveState.DROPBLOCK3;
                }
                break;
            case DROPBLOCK3:
                if (dropTimer.milliseconds() > 50) {
                    //100
                    robot.arm.openClaw();
                    //robot.arm.armPlaceDone();

                    driveState = DriveState.ARMUP3;
                }
                break;
            case ARMUP3:
                if (dropTimer.milliseconds() > 250) {
                    robot.arm.clawWristFront();
                    robot.arm.armTransfer();
                    driveState = DriveState.COMEDOWN3;
                    dropTimer.reset();
                }

                break;
            case COMEDOWN3:
                if (dropTimer.milliseconds() > 175) {
                    //TODO include test that place is done
                    robot.lift.setLiftPos(0); //250
                    //robot.lift.liftFullDown();
                    // robot.lift.setLiftPos(0);
                    /*    if (specimenCount == 5) {
                        if (isBlue) {//TODO: Custom positions
                            double[][] points = {{25, -8}, {55, -12}, {60, -36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};

                            robot.controller.setCurve(points, Math.toRadians(170), 30);

                        } else {
                            double[][] points = {{-25, 8}, {-55, 12}, {-60, 36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                            robot.controller.setCurve(points, Math.toRadians(350), 30);
                        }
                    }

                 */
                    robot.intake.extendLevels(23);
                    robot.intake.wristPickup();

                    //  driveState = DriveState.GOTOBLOCKS;
                    // driveState = DriveState.IDLE;
                    // driveState = DriveState.GETBLOCK3;
                    intakeTimer.reset();
                    intakeState = IntakeState.GETBALLS;
                    driveState = DriveState.ONWAIT3;
                }

                break;
            case ONWAIT3:
                if (intakeState == IntakeState.IDLE) {
                    driveState = DriveState.RETRACT3;
                }
                /* robot.intake.getIntakeProx();
                if (specimenCount > 4) {
                    if (dropTimer.milliseconds() > 500) {//150

                        dropTimer.reset();
                        driveState = DriveState.EXTEND3;
                    }
                } else {
                    if (dropTimer.milliseconds() > 150) {//150

                        dropTimer.reset();
                        driveState = DriveState.EXTEND3;
                    }
                }

                */
                break;
            case RETRACT3:
                if (isRed) {
                    robot.controller.moveToPoint(new Vector2D(56, -56), Math.toRadians(225)); //(57,-57)
                } else {
                    robot.controller.moveToPoint(new Vector2D(-56, 56), Math.toRadians(45));
                }

                if (dropTimer.milliseconds() > 100) {
                    //250

                    robot.intake.wristTransfer();

                    robot.arm.armTransfer();
                    robot.intake.extendTransfer();

                    driveState = DriveState.STARTTRANSFER3;
                    //driveState = DriveState.IDLE;
                    dropTimer.reset();
                }

                break;
            case STARTTRANSFER3:
                //Start moving while transferring

                //robot.intake.extendTransfer();
                if (dropTimer.milliseconds() > 100) {
                    //    robot.controller.moveToPoint(new Vector2D(61, -58), Math.toRadians(225));
                    dropTimer.reset();
                    driveState = DriveState.EXTENDIN3;
                }

                robot.lift.liftFullDown();
                // robot.lift.setLiftPos(0);
                // robot.arm.transferClaw();

                break;
            case EXTENDIN3:
                robot.intake.getExtendProx();

                if (robot.intake.isExtendTouch() || dropTimer.milliseconds() > 1000) {
                    //800//1100
                    driveState = DriveState.ALIGNTRANSFER3;
                    robot.intake.wristTransfer();
                }

                break;
            case ALIGNTRANSFER3: //if we are in transfer position, rotate the wrist to grab the block
                if (robot.lift.isAtPosition() && (dropTimer.milliseconds() > 50)) {
                    //400
                    //   robot.arm.clawWristTransfer();
                    //reset time to give time for servo to move
                    robot.intake.intakeState = Intake.IntakeStates.TRANSFER;
                    if (!robot.intake.isStopperSensor()) {
                        //robot.intake.isIntakeSensor()
                        robot.intake.intakeStopperClosed();
                        robot.intake.intakeTransfer();
                    }
                    dropTimer.reset();
                    driveState = DriveState.TRANSFER3;
                }

                break;
            case TRANSFER3: //close the claw
                //TODO: test the timing
                // robot.intake.intakeStopperClosed();
                robot.intake.intakeTransfer();
                if (
                    (robot.intake.isStopperSensor() && dropTimer.milliseconds() > 135) || dropTimer.milliseconds() > 450
                ) {
                    //350//(!robot.intake.isIntakeSensor()&&dropTimer.milliseconds()>50)//reduced from 300 and 500 because we are already getting
                    //  System.out.println("14423 Sensor "+robot.intake.isIntakeSensor());
                    //  System.out.println("14423 Timer "+dropTimer.milliseconds());
                    //      if (dropTimer.milliseconds()>900){
                    robot.intake.intakeStopperOpen();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.RELEASETRANSFER3; //TODO: Change this to move to position
                }
                break;
            case RELEASETRANSFER3: //spit and raise arm
                if (dropTimer.milliseconds() > 250) {
                    //robot.intake.intakeSpit();
                    dropTimer.reset();
                    //     robot.intake.intakeStopperClosed();
                    driveState = DriveState.LIFTUP3;
                }

                //liftStates = LiftStates.LIFT_UP_RETURN;
                // openState=OpenStates.DONE;
                break;
            case LIFTUP3: //lift up
                if (dropTimer.milliseconds() > 100) {
                    //TODO: Set time
                    robot.intake.intakeTransfer();
                    //   robot.lift.liftTransferAuto();
                    robot.lift.setLiftPos(Lift.LiftLevel.BASKETHIGH.getValue());

                    dropTimer.reset();
                    driveState = DriveState.FINISHTRANSFER3;
                }
                break;
            case FINISHTRANSFER3: //if up, then turn intake off, turn claw wrist out, and come down
                // if (robot.lift.isAtPosition()) {
                if (dropTimer.milliseconds() > 200) {
                    //  robot.intake.intakeOff();
                    //  robot.intake.extendFull();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.arm.clawWristIntake();
                    //pickup=false;
                    // robot.lift.setLiftPos(0);

                    robot.lift.isPlacing = true;
                    robot.arm.closeClawBasket();
                    robot.arm.armPreBasket();
                    //  robot.arm.clawWristBack();
                    robot.intake.wristTransfer();

                    /*  if (isBlue) {
                        robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(45));
                    } else {
                        robot.controller.moveToPoint(new Vector2D(-59,59), Math.toRadians(225));
                    }


                   */
                    driveState = DriveState.TRAJECTORY_4;
                    //driveState = DriveState.IDLE;
                }
                break;
            case TRAJECTORY_4: //specimenCount=3
                robot.arm.armPreBasket();
                robot.arm.clawWristBack();

                //    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                /* if (specimenCount > 3) {
                    robot.arm.armPreBasket();


                }

                */
                // robot.intake.extendLevels(24);
                //robot.arm.clawWristBack();
                // robot.arm.armBasketAuto();

                if (!robot.controller.isDriving() && robot.lift.isAtBasketPosition()) {
                    driveState = DriveState.LETGO4;
                    dropTimer.reset();
                }
                break;
            case LETGO4:
                if (dropTimer.milliseconds() > 100) {
                    //220

                    cycleCount++; //specimenCount=4,5,6

                    // if (dropTimer.milliseconds() > 250) {
                    if (cycleCount < cycles) {
                        robot.arm.armBackBasket();
                        //robot.arm.openClaw();
                        dropTimer.reset();
                        driveState = DriveState.MOVEDROP4;
                    } else {
                        dropTimer.reset();
                        driveState = DriveState.DROPBLOCK4;
                    }
                }
                break;
            case MOVEDROP4:
                if (dropTimer.milliseconds() > 25) {
                    if (isRed) {
                        //TODO: Custom positions
                        switch (cycleCount) {
                            case 1:
                                //     robot.controller.moveToPoint(new Vector2D(60, -48), Math.toRadians(245));//250//240
                                break;
                            case 2:
                                //     robot.controller.moveToPoint(new Vector2D(60, -48), Math.toRadians(272));//270
                                break;
                            case 3:
                                //             robot.controller.moveToPoint(new Vector2D(56, -43), Math.toRadians(305));//304
                                break;
                            case 4:
                                double[][] points = {
                                    { 28, -7 },
                                    { 55, -12 },
                                    { 60, -36 },
                                    {
                                        robot.controller.getController().getPositionVector().x,
                                        robot.controller.getController().getPositionVector().y,
                                    },
                                };

                                robot.controller.setCurve(points, Math.toRadians(180), 30);
                                break;
                            case 5:
                                points = new double[][] {
                                    { 28, -7 },
                                    { 55, -12 },
                                    { 60, -36 },
                                    {
                                        robot.controller.getController().getPositionVector().x,
                                        robot.controller.getController().getPositionVector().y,
                                    },
                                };

                                robot.controller.setCurve(points, Math.toRadians(180), 30);
                                break;
                        }
                    } else {
                        switch (cycleCount) {
                            case 1:
                                //  robot.controller.moveToPoint(new Vector2D(-60, 48), Math.toRadians(66));//-60,48
                                break;
                            case 2:
                                //  robot.controller.moveToPoint(new Vector2D(-60, 48), Math.toRadians(92));
                                break;
                            case 3:
                                //           robot.controller.moveToPoint(new Vector2D(-57, 41), Math.toRadians(127));
                                break;
                            case 4:
                                double[][] points = {
                                    { -28, 7 },
                                    { -55, 12 },
                                    { -60, 36 },
                                    {
                                        robot.controller.getController().getPositionVector().x,
                                        robot.controller.getController().getPositionVector().y,
                                    },
                                };

                                robot.controller.setCurve(points, Math.toRadians(0), 30);
                                break;
                            case 5:
                                points = new double[][] {
                                    { -28, 7 },
                                    { -55, 12 },
                                    { -60, 36 },
                                    {
                                        robot.controller.getController().getPositionVector().x,
                                        robot.controller.getController().getPositionVector().y,
                                    },
                                };

                                robot.controller.setCurve(points, Math.toRadians(0), 30);
                                break;
                        }
                    }
                    //  robot.arm.openClaw(); ///???add here for faster open?
                    dropTimer.reset();
                    driveState = DriveState.DROPBLOCK4;
                }
                break;
            case DROPBLOCK4:
                if (dropTimer.milliseconds() > 150) {
                    //350
                    robot.arm.openClaw();
                    dropTimer.reset();
                    driveState = DriveState.ARMUP4;
                }
                break;
            case ARMUP4:
                //robot.arm.armPlaceDone();
                if (dropTimer.milliseconds() > 300) {
                    robot.arm.clawWristFront();
                    robot.arm.armTransfer();
                    driveState = DriveState.COMEDOWN4;
                    dropTimer.reset();
                }

                break;
            case COMEDOWN4:
                if (dropTimer.milliseconds() > 175) {
                    //TODO include test that place is done
                    robot.lift.setLiftPos(0); //250
                    //robot.lift.liftFullDown();
                    // robot.lift.setLiftPos(0);

                    if (cycleCount < (cycles)) {
                        //  driveState = DriveState.GOTOBLOCKS;
                        // driveState = DriveState.IDLE;

                        driveState = DriveState.GETBLOCKSUB4;
                    } else {
                        robot.arm.armPlaceDone();
                        robot.lift.setLiftPos(0);
                        driveState = DriveState.PARK;
                    }
                }

                break;
            case GETBLOCKSUB4:
                /*   if (robot.controller.remainingDistance < 24) {
                    robot.arm.armTransfer();
                }

              */
                if (!robot.controller.isDriving()) {
                    robot.intake.getIntakeProx();
                    double extend = calculateExtension();
                    //  double off = robot.limelightResult.getTy();//in degrees
                    robot.intake.extendFixed(extend);
                    robot.controller.turnOnly(robot.getOrientation() + Math.toRadians(robot.limelightResult.getTx()));
                    // robot.intake.wristPickup();
                    intakeTimer.reset();
                    intakeState = IntakeState.GETBLOCKSUB;

                    // robot.intake.intakeState = Intake.IntakeStates.ONBASKET;
                    // robot.intake.intakeStopperClosed();
                    //robot.intake.intakeOn();

                    driveState = DriveState.ONWAIT4;
                    dropTimer.reset();
                }
                break;
            case ONWAIT4:
                if (intakeState == IntakeState.IDLE) {
                    if (robot.intake.isStopperSensor()) {
                        driveState = DriveState.RETRACT4;
                    } else {
                        robot.intake.extendFixed(robot.intake.extendoPosition - .35);
                        //robot.controller.turnOnly(robot.getOrientation()+Math.toRadians(10));
                        driveState = DriveState.GETBLOCKSUB4;
                    }
                }
                break;
            case RETRACT4:
                if (dropTimer.milliseconds() > 100) {
                    //250

                    robot.intake.wristHorizontal();

                    robot.arm.armTransfer();
                    robot.intake.extendTransfer();

                    driveState = DriveState.STARTTRANSFER4;
                    //driveState = DriveState.IDLE;
                    dropTimer.reset();
                }

                break;
            case STARTTRANSFER4:
                //Start moving while transferring
                if (dropTimer.milliseconds() > 400) {
                    //robot.intake.extendTransfer();
                    if (isRed) {
                        //500 //59,-56, 60,-36
                        double[][] points = {
                            { 57, -57 },
                            { 58, -36 },
                            { 48, 0 },
                            {
                                robot.controller.getController().getPositionVector().x,
                                robot.controller.getController().getPositionVector().y,
                            },
                        };
                        robot.controller.setCurve(points, Math.toRadians(225), 30);
                        robot.intake.wristTransfer();
                        dropTimer.reset();
                        robot.intake.extendTransfer();
                        driveState = DriveState.EXTENDIN4;
                    }
                    // robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(225));
                    else {
                        //isRed
                        double[][] points = {
                            { -57, 57 },
                            { -58, 36 },
                            { -48, 0 },
                            {
                                robot.controller.getController().getPositionVector().x,
                                robot.controller.getController().getPositionVector().y,
                            },
                        };
                        robot.controller.setCurve(points, Math.toRadians(45), 30); //45
                        robot.intake.extendTransfer();
                        robot.intake.wristTransfer();
                        dropTimer.reset();
                        driveState = DriveState.EXTENDIN4;
                    }
                    // robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(225));
                    //    }
                }
                robot.lift.liftFullDown();
                // robot.lift.setLiftPos(0);
                // robot.arm.transferClaw();

                break;
            case EXTENDIN4:
                robot.intake.getExtendProx();

                if (robot.intake.isExtendTouch() || dropTimer.milliseconds() > 4000) {
                    //800//1100
                    driveState = DriveState.ALIGNTRANSFER4;
                    robot.intake.wristTransfer();
                }
                break;
            case ALIGNTRANSFER4: //if we are in transfer position, rotate the wrist to grab the block
                if (robot.lift.isAtPosition() && (dropTimer.milliseconds() > 50)) {
                    //400
                    //   robot.arm.clawWristTransfer();
                    //reset time to give time for servo to move
                    robot.intake.intakeState = Intake.IntakeStates.TRANSFER;
                    if (!robot.intake.isStopperSensor()) {
                        //robot.intake.isIntakeSensor()
                        robot.intake.intakeStopperClosed();
                        robot.intake.intakeTransfer();
                    }
                    dropTimer.reset();
                    driveState = DriveState.TRANSFER4;
                }

                break;
            case TRANSFER4: //close the claw
                //TODO: test the timing
                // robot.intake.intakeStopperClosed();
                robot.intake.intakeTransfer();
                if (
                    (robot.intake.isStopperSensor() && dropTimer.milliseconds() > 135) || dropTimer.milliseconds() > 450
                ) {
                    //350//(!robot.intake.isIntakeSensor()&&dropTimer.milliseconds()>50)//reduced from 300 and 500 because we are already getting
                    //  System.out.println("14423 Sensor "+robot.intake.isIntakeSensor());
                    //  System.out.println("14423 Timer "+dropTimer.milliseconds());
                    //      if (dropTimer.milliseconds()>900){
                    robot.intake.intakeStopperOpen();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.RELEASETRANSFER4; //TODO: Change this to move to position
                }
                break;
            case RELEASETRANSFER4: //spit and raise arm
                if (dropTimer.milliseconds() > 250) {
                    //robot.intake.intakeSpit();
                    dropTimer.reset();
                    //     robot.intake.intakeStopperClosed();
                    driveState = DriveState.LIFTUP4;
                }

                //liftStates = LiftStates.LIFT_UP_RETURN;
                // openState=OpenStates.DONE;
                break;
            case LIFTUP4: //lift up
                if (dropTimer.milliseconds() > 200) {
                    //TODO: Set time
                    robot.intake.intakeTransfer();
                    //robot.lift.liftTransferAuto();
                    robot.lift.setLiftPos(Lift.LiftLevel.BASKETHIGH.getValue());

                    dropTimer.reset();
                    driveState = DriveState.FINISHTRANSFER4;
                }
                break;
            case FINISHTRANSFER4: //if up, then turn intake off, turn claw wrist out, and come down
                // if (robot.lift.isAtPosition()) {
                if (dropTimer.milliseconds() > 200) {
                    //  robot.intake.intakeOff();
                    //  robot.intake.extendFull();
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.arm.clawWristIntake();
                    //pickup=false;
                    // robot.lift.setLiftPos(0);

                    robot.lift.isPlacing = true;
                    robot.arm.closeClawBasket();
                    robot.arm.armPreBasket();
                    robot.arm.clawWristBack();
                    robot.intake.wristTransfer();

                    /*  if (isBlue) {
                        robot.controller.moveToPoint(new Vector2D(59,-59), Math.toRadians(45));
                    } else {
                        robot.controller.moveToPoint(new Vector2D(-59,59), Math.toRadians(225));
                    }


                   */
                    driveState = DriveState.TRAJECTORY_4;
                }
                break;

        }
    }

    public void dumpStart() {
        //TODO:SET VISION TO PROPER APRILTAG TARGET
        //  robot.arm.clawWristBack();
        //  robot.arm.closeClawSpecimen();
        //robot.arm.clawWristFront();
      /*
        if (isRed) {
            robot.controller.moveToPointRam(new Vector2D(5, -31), Math.toRadians(270)); //28.5//-5//state was 29.5
        } else {
            robot.controller.moveToPointRam(new Vector2D(-5, 31), Math.toRadians(90));
        }

       */

    }

    public void specimen() {
        //  robot.controller.update(robot.getOrientation());
        switch (driveState) {
            case IDLE:
                break;
            case SHOOT_1:
                robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                if (robot.controller.remainingDistance < 12) {
                    robot.arm.closeStopper();
                }

                if (Math.abs(robot.controller.remainingDistance) < 3) {
                    //3 //consider less if we are driving further out?
                    driveState = DriveState.DROP;
                }

                break;
            case DROP:
                //   robot.purpleOpen();
                dropTimer.reset();
                //commented out if ramming
                robot.lift.liftPlace();
                //  robot.lift.liftPlaceAuto();
                driveState = DriveState.WAIT_1;
                /*  switch (path) {
                    case BASKET:
                        driveState = DriveState.DONE_DROP;
                        break;
                    case SPECIMEN:
                        driveState = DriveState.TRAJECTORY_5;
                        break;
                    case STOP:
                        driveState = DriveState.BACKOUT;
                        break;
                }
               */

                break;
            case WAIT_1:
                if (robot.lift.isAtPosition() || dropTimer.milliseconds() > 450) {
                    if (robot.controller.isDriving()) {
                        //||robot.controller.noMoveCycles>30
                        robot.controller.abort = true;
                    }
                    robot.arm.openStopper();
                    robot.arm.openClaw();
                    dropTimer.reset();
                    driveState = DriveState.PATH_2;
                }

                // if (robot.lift.isAtPosition()) {
                /*
                if (robot.controller.isNotMoving()){
                   if(notMoving){
                       if (dropTimer.milliseconds()>450){ //500
                           donePlace=true;
                       }
                   } else {
                       dropTimer.reset();
                        notMoving=true;
                    }
                }


            //   if (!robot.controller.isDriving()||robot.controller.noMoveCycles>30) {
             //       if (robot.controller.isDriving()||robot.controller.noMoveCycles>30) {
                if (!robot.controller.isDriving()||donePlace) {
                    if (robot.controller.isDriving()||donePlace) {
                        robot.controller.abort = true;
                    }
                   //         donePlace=false;
                     //       notMoving=false;
                    robot.arm.openClaw();
                   // robot.lift.liftPlace();//Include here only if ramming
                    robot.arm.openStopper();

                    dropTimer.reset();
                    driveState = DriveState.ARMUP;
                }

             */
                break;
            case PATH_2:
                if (dropTimer.milliseconds() > 100) {
                    //robot.arm.armPlaceDone();
                    robot.arm.armPickup();
                    driveState = DriveState.PATH_3;
                    dropTimer.reset();
                }
                /*
                if (dropTimer.milliseconds() >100) {
                    robot.arm.armPlaceDone();
                    //  robot.arm.armPickup();
                    driveState = DriveState.COMEDOWN;
                     dropTimer.reset();
                }

               */
                break;
            case PATH_3:
                if (dropTimer.milliseconds() > 100) {
                    //150
                    //TODO include test that place is done
                    //robot.lift.liftFullDown();
                    robot.lift.setLiftPos(0);

                    //   driveState = DriveState.DROP;

                    // else

                    //if(!robot.controller.isDriving()){

                    //  driveState = DriveState.DROP;
                    // }
                    cycleCount++;
                    if (cycleCount == 1) {
                        driveState = DriveState.GOTOBLOCKS;
                    } else {
                        if (cycleCount < cycles) {
                            //robot.arm.armPickup();

                            driveState = DriveState.ALIGN;
                        } else {
                            robot.arm.armPreBasket();
                            // robot.lift.setLiftPos(0);
                            driveState = DriveState.PARK;
                        }
                    }
                }
                break;
            case GOTOBLOCKS:
                //   if(robot.lift.isAtPosition()){//&&robot.arm.isArmAtTransfer()
                //  robot.arm.armTransfer();

                //ANGLE 235 .52 //217 max.436
                if (isRed) {
                    double[][] points = {
                        { -50, -44 },
                        { -34, -47 },
                        { -7, -45 },
                        {
                            robot.controller.getController().getPositionVector().x,
                            robot.controller.getController().getPositionVector().y,
                        },
                    };
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(267), 30); //222
                } else {
                    double[][] points = {
                        { 50, 44 },
                        { 34, 47 },
                        { 7, 45 },
                        {
                            robot.controller.getController().getPositionVector().x,
                            robot.controller.getController().getPositionVector().y,
                        },
                    };
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(88), 30);
                }

                /* STRAIGHT
                    double[][] points = {{-36 ,-29}, {-25,-45}, {-6,-51},{robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y}};
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(193), 30);

                     */
                // robot.intake.extendLevels(9 );
                // robot.intake.wristPickup();
                driveState = DriveState.GETBLOCK;
                //    }
                break;
            case GETBLOCK:
                if (robot.controller.remainingDistance < 12) {
                    robot.intake.extendLevels(16); //15
                    intakeTimer.reset();
                    intakeState = IntakeState.GETBLOCKSPECIMEN; //6
                }
                if (!robot.controller.isDriving()) {
                    //  robot.intake.intakeState= Intake.IntakeStates.ONAUTO;

                    //robot.intake.intakeOn();

                    driveState = DriveState.INTAKE_2;
                    // driveState = DriveState.IDLE;
                    dropTimer.reset();
                }
                break;
            case INTAKE_2:
                if (intakeState == IntakeState.IDLE) {
                    //    System.out.println("14423 intake is IDLE");
                    driveState = DriveState.TURNBLOCK;
                }
                break;
            case TURNBLOCK:
                // System.out.println("14423 in TURNBLOCK");
                //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                if (isRed) {
                    robot.controller.turnOnlyFast(Math.toRadians(130));
                } else {
                    robot.controller.turnOnlyFast(Math.toRadians(310));
                }
                // robot.controller.turnOnly(Math.toRadians(120));
                robot.intake.extendLevels(10);

                // robot.intake.wristSpit();
                driveState = DriveState.PATH_1;
                break;
            case PATH_1:
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(30))) {
                    //15
                    robot.intake.wristSpit();
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;
                    dropTimer.reset();

                    driveState = DriveState.EXTENDTIME;
                }
                /* don't need to wait until done driving
              if(!robot.controller.isDriving()){

                    //robot.intake.intakeSpit();
                    dropTimer.reset();

                    driveState = DriveState.EXTENDTIME;
                }

              */
                break;
            case EXTENDTIME:
                if (
                    !robot.intake.isIntakeSensor() & (dropTimer.milliseconds() > 400) || dropTimer.milliseconds() > 950
                ) {
                    //450
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.intake.extendLevels(16);//19

                    dropTimer.reset();
                    driveState = DriveState.TURN2;
                }
                break;
            case TURN2:
                if (dropTimer.milliseconds() > 0) {
                    //20

                    // robot.intake.intakeState= Intake.IntakeStates.ONAUTO;
                    if (isRed) {
                        robot.controller.turnOnly(Math.toRadians(232)); //200//204//180+24
                    } else {
                        robot.controller.turnOnly(Math.toRadians(52)); //180-24
                    }
                    // robot.controller.turnOnly(Math.toRadians(190));

                    robot.intake.wristPickup();

                    driveState = DriveState.GETBLOCK2;
                }
                break;
            case GETBLOCK2:
                robot.intake.getIntakeProx();
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(50))) {
                    //15
                    robot.intake.extendLevels(19);
                    intakeTimer.reset();
                    intakeState = IntakeState.GETBLOCKSPECIMEN; //6
                    driveState = DriveState.ONWAIT2;
                    dropTimer.reset();
                }
                /* Don't need to wait until stop turning if we get it
             if(!robot.controller.isDriving()){
                    //robot.intake.extendLevels(5);



                    //robot.intake.intakeOn();

                    driveState = DriveState.ONWAIT2;
                    dropTimer.reset();
                }

              */
                break;
            case ONWAIT2:
                if (intakeState == IntakeState.IDLE) {
                    robot.intake.wristSpit();

                    driveState = DriveState.TURNBLOCK2;
                }
                break;
            case TURNBLOCK2:
                //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                if (isRed) {
                    robot.controller.turnOnlyFast(Math.toRadians(130));
                } else {
                    robot.controller.turnOnlyFast(Math.toRadians(310));
                }
                //  robot.controller.turnOnly(Math.toRadians(110));

                robot.intake.extendLevels(19);

                robot.intake.wristSpit();
                driveState = DriveState.DROPBLOCK2;
                break;
            case DROPBLOCK2:
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(25))) {
                    //15
                    //    robot.intake.wristSpit();
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;
                    dropTimer.reset();
                    driveState = DriveState.EXTENDTIME2;
                }
                /*
                if(!robot.controller.isDriving()){

                    //robot.intake.intakeState= Intake.IntakeStates.SPIT;
                    dropTimer.reset();

                    driveState = DriveState.TURN3;
       //             driveState = DriveState.STARTPICKUP2;
                }

              */
                break;
            case EXTENDTIME2:
                if (
                    !robot.intake.isIntakeSensor() & (dropTimer.milliseconds() > 400) || dropTimer.milliseconds() > 950
                ) {
                    //450
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    // robot.intake.extendLevels(16);//19

                    dropTimer.reset();
                    driveState = DriveState.TURN3;
                }
                break;
            case TURN3:
                if (dropTimer.milliseconds() > 0) {
                    //20
                    if (isRed) {
                        robot.controller.turnOnly(Math.toRadians(210)); //216
                    } else {
                        robot.controller.turnOnly(Math.toRadians(30)); //180-24
                    }
                    //     robot.controller.moveToPoint(new Vector2D(-50,-29),Math.toRadians(190));
                    //robot.controller.turnOnly(Math.toRadians(212));
                    //robot.intake.extendLevels(16);//12
                    driveState = DriveState.GETBLOCK3;
                }
                break;
            case GETBLOCK3:
                robot.intake.getIntakeProx();
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(20))) {
                    //15
                    robot.intake.extendFull();
                    intakeTimer.reset();
                    intakeState = IntakeState.GETBLOCKSPECIMEN; //6
                    driveState = DriveState.ONWAIT3;
                    dropTimer.reset();
                }

                /* Don't need to wait
                if(!robot.controller.isDriving()){
                    //robot.intake.extendLevels(5);
                   // robot.intake.intakeState= Intake.IntakeStates.ONWAIT3;
                    //robot.intake.wristPickup();
                    //robot.intake.intakeOn();

                    driveState = DriveState.ONWAIT3;
                    dropTimer.reset();
                }

                */
                break;
            case ONWAIT3:
                if (intakeState == IntakeState.IDLE) {
                    driveState = DriveState.TURNBLOCK3;
                }
                break;
            /*    case EXTEND3:
                if (dropTimer.milliseconds()>20) {
                    if (!(robot.intake.intakeState== Intake.IntakeStates.ACQUIRED) && robot.intake.extendo.getPosition()<robot.intake.MAXEXTEND ) {
                        robot.intake.extendoOut();
                        dropTimer.reset();
                    } else {

                        dropTimer.reset();
                        robot.intake.intakeState= Intake.IntakeStates.OFF;

                        //TODO: Code to get blocks with intake }
                        driveState = DriveState.RETRACT3;
                    } }
                break;
            case RETRACT3:
                if (dropTimer.milliseconds()>200){
                    robot.intake.wristTransfer();
                    robot.intake.extendLevels(4);
                    dropTimer.reset();
                    driveState = DriveState.TURNBLOCK3;
                }

                break;

         */
            case TURNBLOCK3:
                //     robot.intake.extendLevels(6);
                if (dropTimer.milliseconds() > 10) {
                    //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                    if (isRed) {
                        robot.controller.turnOnlyFast(Math.toRadians(130));
                    } else {
                        robot.controller.turnOnlyFast(Math.toRadians(310));
                    }

                    //  robot.controller.turnOnly(Math.toRadians(90) );
                    dropTimer.reset();
                    driveState = DriveState.EXTENDBACK;
                }
                break;
            case EXTENDBACK:
                if (dropTimer.milliseconds() > 10) {
                    robot.intake.extendLevels(16);

                    driveState = DriveState.DROPBLOCK3;
                }
                break;
            case DROPBLOCK3:
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(30))) {
                    //15
                    robot.intake.wristSpit();
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;

                    // robot.intake.intakeSpit();
                    dropTimer.reset();
                    //TODO: Code to drop blocks with intake
                    driveState = DriveState.STARTPICKUP2;
                }
                break;
            case STARTPICKUP2:
                if (
                    (!robot.intake.isIntakeSensor() && dropTimer.milliseconds() > 400) || dropTimer.milliseconds() > 750
                ) {
                    //450
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.intake.wristHorizontal();
                    robot.intake.extendTransfer();

                    if (isRed) {
                        robot.controller.turnOnlyFast(Math.toRadians(270)); //testing - may want sloser
                    } else {
                        robot.controller.turnOnlyFast(Math.toRadians(90));
                    }
                    //robot.intake.extendMin();
                    robot.arm.openClaw();
                    robot.arm.armPickup();
                    driveState = DriveState.ALIGN;
                }
                break;
            case ALIGN:
                if (!robot.controller.isDriving()) {
                    //                    double[][] points = {{-37, -61}, {-40, -51}, {-40, -36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    //    double[][] points = {{-34.88, -62.5}, {-40, -51}, {-40, -36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    driveState = DriveState.STARTDRIVING;
                    dropTimer.reset();
                    //   robot.controller.setCurve(points, Math.toRadians(270), 30);
                }
                break;
            case STARTDRIVING:
                /*     if (robot.controller.getController().getSparkVector().y<-48) {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.ULTRASONIC);
                } else {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                }

            */

                //  if (dropTimer.milliseconds()>100) {

                if (isRed) {
                    //    if (specimenCount==1) {
                    robot.controller.moveToPointPrecise(new Vector2D(-35.25, -60.75), Math.toRadians(270)); //-61.75//-62
                    //  } else {
                    //    double[][] points = {{-35.25, -61}, {-34, -47.5}, {-8, -47.5}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    //   robot.controller.setCurvePrecise(points, Math.toRadians(270), 30);
                    //  }
                } else {
                    robot.controller.moveToPointPrecise(new Vector2D(35.25, 60.75), Math.toRadians(90));
                }
                robot.arm.openClaw();
                robot.arm.armPickup();
                driveState = DriveState.CLOSECLAW;
                //    }
                break;
            case CLOSECLAW:
                if (
                    Math.abs(robot.controller.getController().getSparkVector().y) > 48 &&
                    Math.abs(robot.controller.getController().getSparkVector().x) > 24
                ) {
                  //  robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.ULTRASONIC);
                } else {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                }
                // updatePoseUltrasonic();
                // if (!robot.controller.isDriving()) {//speed up grab by using distance
                if (robot.controller.remainingDistance < .5) {
                    //   robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.STARTLIFT2;
                    //   driveState = DriveState.IDLE;
                }
                break;
            case STARTLIFT2:
                if (dropTimer.milliseconds() > 200) {
                    //250
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    robot.lift.setLiftPos(Lift.LiftLevel.SPECIMENHIGH.getValue());
                    robot.lift.isPlacing = true;
                    if (isRed) {
                        //     double[][] points = {{2.5-((specimenCount-2)*2.5),-32}, {2.5-((specimenCount-2)*2.5), -42}, {-3.3, -45}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                        //    robot.controller.setCurveRam(points, Math.toRadians(270), 30);
                        //    robot.controller.moveToPointRam(new Vector2D(-2.5+((specimenCount-2)*2.5),-31), Math.toRadians(270));//1.5//28.5//3.5, 5.25,29.5

                        // if (specimenCount>3 ){
                        //   robot.controller.moveToPoint(new Vector2D(2.5 - ((specimenCount - 2) * 2.5), -32), Math.toRadians(270));
                        //} else {
                        robot.controller.moveToPointRam(
                            new Vector2D(2.5 - ((cycleCount - 2) * 2.5), -32),
                            Math.toRadians(270)
                        ); //1.5//28.5//3.5, 5.25,29.5 //state was 29.5
                        // }
                    } else {
                        robot.controller.moveToPointRam(
                            new Vector2D(-2.5 + ((cycleCount - 2) * 2.5), 32),
                            Math.toRadians(90)
                        ); //-1.5
                    }
                    robot.arm.armPlace();
                    driveState = DriveState.STARTPLACE2;
                }
                break;
            case STARTPLACE2:
                if (dropTimer.milliseconds() > 100) {
                    //400
                    //  robot.arm.armPlace();
                    //  robot.intake.wristTransfer();
                    /* if (isBlue) {
                   //     double[][] points = {{2.5-((specimenCount-2)*2.5),-32}, {2.5-((specimenCount-2)*2.5), -42}, {-3.3, -45}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    //    robot.controller.setCurveRam(points, Math.toRadians(270), 30);
                    //    robot.controller.moveToPointRam(new Vector2D(-2.5+((specimenCount-2)*2.5),-31), Math.toRadians(270));//1.5//28.5//3.5, 5.25,29.5
                        robot.controller.moveToPointRam(new Vector2D(2.5-((specimenCount-2)*2.5),-32), Math.toRadians(270));//1.5//28.5//3.5, 5.25,29.5 //state was 29.5
                    } else {
                        robot.controller.moveToPointRam(new Vector2D(-2.5 + ((specimenCount-2)*2.5),32 ), Math.toRadians(90));//-1.5
                    }

                    */
                    //noMoveCycles=0;
                    driveState = DriveState.SHOOT_1;
                }
                break;
            case PARK:
                if (isRed) {
                    robot.controller.moveToPointPrecise(new Vector2D(-36.25, -61.75), Math.toRadians(270));
                } else {
                    robot.controller.moveToPointPrecise(new Vector2D(36.25, 61.75), Math.toRadians(90));
                }
                break;
        }
    }

    public void specimenState() {
        //  robot.controller.update(robot.getOrientation());
        switch (driveState) {
            case IDLE:
                break;
            case SHOOT_1:
                robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                if (robot.controller.remainingDistance < 10) {
                    robot.arm.closeStopper();
                }

                if (robot.controller.remainingDistance < 3) {
                    driveState = DriveState.DROP;
                }

                break;
            case DROP:
                //   robot.purpleOpen();
                dropTimer.reset();
                //   robot.lift.liftPlace();

                driveState = DriveState.WAIT_1;
                /*  switch (path) {
                    case BASKET:
                        driveState = DriveState.DONE_DROP;
                        break;
                    case SPECIMEN:
                        driveState = DriveState.TRAJECTORY_5;
                        break;
                    case STOP:
                        driveState = DriveState.BACKOUT;
                        break;
                }
               */

                break;
            case WAIT_1:
                // if (robot.lift.isAtPosition()) {
                if (!robot.controller.isDriving() || robot.controller.noMoveCycles > 30) {
                    if (robot.controller.isDriving() || robot.controller.noMoveCycles > 30) {
                        robot.controller.abort = true;
                    }
                    robot.arm.openStopper();
                    robot.arm.openClaw();
                    dropTimer.reset();
                    driveState = DriveState.PATH_2;
                }
                break;
            case PATH_2:
                if (dropTimer.milliseconds() > 100) {
                    //robot.arm.armPlaceDone();
                    //  robot.arm.armPickup();
                    driveState = DriveState.PATH_3;
                    dropTimer.reset();
                }
                break;
            case PATH_3:
                if (dropTimer.milliseconds() > 25) {
                    //150
                    //TODO include test that place is done
                    //robot.lift.liftFullDown();
                    robot.lift.setLiftPos(0);

                    //   driveState = DriveState.DROP;

                    // else

                    //if(!robot.controller.isDriving()){

                    //  driveState = DriveState.DROP;
                    // }
                    cycleCount++;
                    if (cycleCount == 1) {
                        driveState = DriveState.GOTOBLOCKS;
                    } else {
                        if (cycleCount == 2 || cycleCount == 3) {
                            driveState = DriveState.ALIGN;
                        } else {
                            robot.arm.armPreBasket();
                            // robot.lift.setLiftPos(0);
                            driveState = DriveState.PARK;
                        }
                    }
                }
                break;
            case GOTOBLOCKS:
                //   if(robot.lift.isAtPosition()){//&&robot.arm.isArmAtTransfer()
                //  robot.arm.armTransfer();

                //ANGLE
                if (isRed) {
                    double[][] points = {
                        { -39, -34 },
                        { -34, -47 },
                        { -7, -45 },
                        {
                            robot.controller.getController().getPositionVector().x,
                            robot.controller.getController().getPositionVector().y,
                        },
                    };
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(214), 30); //222
                } else {
                    double[][] points = {
                        { 39, 34 },
                        { 34, 47 },
                        { 7, 45 },
                        {
                            robot.controller.getController().getPositionVector().x,
                            robot.controller.getController().getPositionVector().y,
                        },
                    };
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(34), 30);
                }

                /* STRAIGHT
                    double[][] points = {{-36 ,-29}, {-25,-45}, {-6,-51},{robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y}};
                    //driveState=DriveState.IDLE;
                    robot.controller.setCurve(points, Math.toRadians(193), 30);

                     */
                // robot.intake.extendLevels(9 );
                // robot.intake.wristPickup();
                driveState = DriveState.GETBLOCK;
                //    }
                break;
            case GETBLOCK:
                if (robot.controller.remainingDistance < 12) {
                    robot.intake.extendLevels(3); //6
                }
                if (!robot.controller.isDriving()) {
                    //  robot.intake.intakeState= Intake.IntakeStates.ONAUTO;

                    //robot.intake.intakeOn();

                    driveState = DriveState.INTAKE_2;
                    dropTimer.reset();
                }
                break;
            case INTAKE_2:
                if (dropTimer.milliseconds() > 50) {
                    robot.intake.wristPickup();
                    robot.intake.intakeState = Intake.IntakeStates.ONAUTO;

                    dropTimer.reset();
                    driveState = DriveState.EXTEND;
                }
                break;
            case EXTEND:
                if (dropTimer.milliseconds() > 20) {
                    if (
                        !(robot.intake.intakeState == Intake.IntakeStates.GETTINGAUTO ||
                            robot.intake.intakeState == Intake.IntakeStates.ACQUIRED) &&
                        robot.intake.extendo.getPosition() < robot.intake.MAXEXTEND
                    ) {
                        robot.intake.extendoOutFast();
                        dropTimer.reset();
                    } else {
                        dropTimer.reset();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        robot.intake.extendLevels(4); //4
                        //robot.intake.wristTransfer();
                        //TODO: Code to get blocks with intake }
                        driveState = DriveState.PATH_4a;
                    }
                }
                break;
            case PATH_4a:
                if (dropTimer.milliseconds() > 200) {
                    driveState = DriveState.TURNBLOCK;
                }

                break;
            case TURNBLOCK:
                //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                if (isRed) {
                    robot.controller.turnOnly(Math.toRadians(130));
                } else {
                    robot.controller.turnOnly(Math.toRadians(310));
                }
                // robot.controller.turnOnly(Math.toRadians(120));
                robot.intake.extendFull();
                robot.intake.wristPickup();
                // robot.intake.wristSpit();
                driveState = DriveState.PATH_1;
                break;
            case PATH_1:
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(15))) {
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;
                }
                if (!robot.controller.isDriving()) {
                    //robot.intake.intakeSpit();
                    dropTimer.reset();
                    //TODO: Code to drop blocks with intake
                    driveState = DriveState.EXTENDTIME;
                }
                break;
            case EXTENDTIME:
                if (
                    !robot.intake.isIntakeSensor() & (dropTimer.milliseconds() > 450) || dropTimer.milliseconds() > 950
                ) {
                    //750
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.intake.extendLevels(16); //19

                    dropTimer.reset();
                    driveState = DriveState.TURN2;
                }
                break;
            case TURN2:
                if (dropTimer.milliseconds() > 20) {
                    //200

                    // robot.intake.intakeState= Intake.IntakeStates.ONAUTO;
                    if (isRed) {
                        robot.controller.turnOnly(Math.toRadians(198)); //200//204//180+24
                    } else {
                        robot.controller.turnOnly(Math.toRadians(18)); //180-24
                    }
                    // robot.controller.turnOnly(Math.toRadians(190));

                    robot.intake.wristPickup();

                    driveState = DriveState.GETBLOCK2;
                }
                break;
            case GETBLOCK2:
                robot.intake.getIntakeProx();

                if (!robot.controller.isDriving()) {
                    //robot.intake.extendLevels(5);

                    //robot.intake.intakeOn();
                    robot.intake.intakeState = Intake.IntakeStates.ONAUTO;
                    driveState = DriveState.EXTEND2;
                    dropTimer.reset();
                }
                break;
            case EXTEND2:
                if (dropTimer.milliseconds() > 20) {
                    if (
                        !(robot.intake.intakeState == Intake.IntakeStates.GETTINGAUTO ||
                            robot.intake.intakeState == Intake.IntakeStates.ACQUIRED) &&
                        robot.intake.extendo.getPosition() < robot.intake.MAXEXTEND
                    ) {
                        robot.intake.extendoOutFast();
                        dropTimer.reset();
                    } else {
                        dropTimer.reset();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;
                        //robot.intake.wristTransfer();
                        robot.intake.extendLevels(8); //10
                        //TODO: Code to get blocks with intake }
                        driveState = DriveState.RETRACT2;
                    }
                }
                break;
            case RETRACT2:
                if (dropTimer.milliseconds() > 200) {
                    driveState = DriveState.TURNBLOCK2;
                }

                break;
            case TURNBLOCK2:
                //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                if (isRed) {
                    robot.controller.turnOnly(Math.toRadians(130));
                } else {
                    robot.controller.turnOnly(Math.toRadians(310));
                }
                //  robot.controller.turnOnly(Math.toRadians(110));

                robot.intake.extendFull();
                robot.intake.wristPickup();
                //robot.intake.wristSpit();
                driveState = DriveState.DROPBLOCK2;
                break;
            case DROPBLOCK2:
                if (robot.controller.remainingAngle < Math.abs(Math.toRadians(15))) {
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;
                }
                if (!robot.controller.isDriving()) {
                    //robot.intake.intakeState= Intake.IntakeStates.SPIT;
                    dropTimer.reset();
                    //TODO: Code to drop blocks with intake
                    // driveState = DriveState.TURN3;
                    driveState = DriveState.STARTPICKUP2;
                }
                break;
            case TURN3:
                if (dropTimer.milliseconds() > 350) {
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    if (isRed) {
                        robot.controller.moveToPoint(new Vector2D(-50, -39), Math.toRadians(212));
                    } else {
                        robot.controller.moveToPoint(new Vector2D(50, 39), Math.toRadians(32));
                    }
                    //     robot.controller.moveToPoint(new Vector2D(-50,-29),Math.toRadians(190));
                    //robot.controller.turnOnly(Math.toRadians(212));
                    robot.intake.extendLevels(10); //12
                    driveState = DriveState.GETBLOCK3;
                }
                break;
            case GETBLOCK3:
                robot.intake.getIntakeProx();
                if (!robot.controller.isDriving()) {
                    //robot.intake.extendLevels(5);
                    robot.intake.intakeState = Intake.IntakeStates.ONAUTO;
                    robot.intake.wristPickup();
                    //robot.intake.intakeOn();

                    driveState = DriveState.EXTEND3;
                    dropTimer.reset();
                }
                break;
            case EXTEND3:
                if (dropTimer.milliseconds() > 20) {
                    if (
                        !(robot.intake.intakeState == Intake.IntakeStates.ACQUIRED) &&
                        robot.intake.extendo.getPosition() < robot.intake.MAXEXTEND
                    ) {
                        robot.intake.extendoOut();
                        dropTimer.reset();
                    } else {
                        dropTimer.reset();
                        robot.intake.intakeState = Intake.IntakeStates.OFF;

                        //TODO: Code to get blocks with intake }
                        driveState = DriveState.RETRACT3;
                    }
                }
                break;
            case RETRACT3:
                if (dropTimer.milliseconds() > 200) {
                    robot.intake.wristTransfer();
                    robot.intake.extendLevels(4);
                    dropTimer.reset();
                    driveState = DriveState.TURNBLOCK3;
                }

                break;
            case TURNBLOCK3:
                if (dropTimer.milliseconds() > 250) {
                    //  robot.controller.moveToPoint(new Vector2D(robot.controller.getController().getPositionVector().x,robot.controller.getController().getPositionVector().y), Math.toRadians(135));
                    if (isRed) {
                        robot.controller.turnOnly(Math.toRadians(130));
                    } else {
                        robot.controller.turnOnly(Math.toRadians(310));
                    }

                    //  robot.controller.turnOnly(Math.toRadians(90) );
                    dropTimer.reset();
                    driveState = DriveState.EXTENDBACK;
                }
                break;
            case EXTENDBACK:
                if (dropTimer.milliseconds() > 150) {
                    robot.intake.extendFull();
                    robot.intake.wristSpit();
                    driveState = DriveState.DROPBLOCK3;
                }
                break;
            case DROPBLOCK3:
                if (!robot.controller.isDriving()) {
                    robot.intake.intakeState = Intake.IntakeStates.SPIT;
                    // robot.intake.intakeSpit();
                    dropTimer.reset();
                    //TODO: Code to drop blocks with intake
                    //   driveState = DriveState.TURN2;
                }
                break;
            case STARTPICKUP2:
                if (
                    (!robot.intake.isIntakeSensor() && dropTimer.milliseconds() > 450) || dropTimer.milliseconds() > 750
                ) {
                    //700
                    robot.intake.intakeState = Intake.IntakeStates.OFF;
                    robot.intake.extendTransfer();
                    robot.intake.wristTransfer();
                    if (isRed) {
                        robot.controller.turnOnly(Math.toRadians(270));
                    } else {
                        robot.controller.turnOnly(Math.toRadians(90));
                    }
                    //robot.intake.extendMin();
                    robot.arm.openClaw();
                    driveState = DriveState.ALIGN;
                }
                break;
            case ALIGN:
                if (!robot.controller.isDriving()) {
                    //                    double[][] points = {{-37, -61}, {-40, -51}, {-40, -36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    //    double[][] points = {{-34.88, -62.5}, {-40, -51}, {-40, -36}, {robot.controller.getController().getPositionVector().x, robot.controller.getController().getPositionVector().y}};
                    driveState = DriveState.STARTDRIVING;
                    dropTimer.reset();
                    //   robot.controller.setCurve(points, Math.toRadians(270), 30);
                }
                break;
            case STARTDRIVING:
                /*     if (robot.controller.getController().getSparkVector().y<-48) {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.ULTRASONIC);
                } else {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                }

            */

                //  if (dropTimer.milliseconds()>100) {

                if (isRed) {
                    robot.controller.moveToPointPrecise(new Vector2D(-35.25, -62), Math.toRadians(270)); //-61.75
                } else {
                    robot.controller.moveToPointPrecise(new Vector2D(35.25, 62), Math.toRadians(90));
                }

                driveState = DriveState.CLOSECLAW;
                //    }
                break;
            case CLOSECLAW:
                if (
                    Math.abs(robot.controller.getController().getSparkVector().y) > 48 &&
                    Math.abs(robot.controller.getController().getSparkVector().x) > 24
                ) {
                 //   robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.ULTRASONIC);
                } else {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                }
                // updatePoseUltrasonic();
                if (!robot.controller.isDriving()) {
                    robot.controller.getController().setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);
                    robot.arm.closeClaw();
                    dropTimer.reset();
                    driveState = DriveState.STARTLIFT2;
                    //   driveState = DriveState.IDLE;
                }
                break;
            case STARTLIFT2:
                if (dropTimer.milliseconds() > 250) {
                    //350
                    robot.lift.setLiftPos(Lift.LiftLevel.SPECIMENHIGH.getValue());
                    robot.lift.isPlacing = true;

                    driveState = DriveState.STARTPLACE2;
                }
                break;
            case STARTPLACE2:
                if (dropTimer.milliseconds() > 250) {
                    //400
                    robot.arm.armPlace();
                    robot.intake.wristTransfer();
                    if (isRed) {
                        robot.controller.moveToPoint(
                            new Vector2D(3.5 + ((cycleCount - 2) * 5.25), -29.5),
                            Math.toRadians(270)
                        ); //4, 5.25
                    } else {
                        robot.controller.moveToPoint(
                            new Vector2D(-3.5 - ((cycleCount - 2) * 5.25), 29.5),
                            Math.toRadians(90)
                        );
                    }
                    //noMoveCycles=0;
                    driveState = DriveState.SHOOT_1;
                }
                break;
            case PARK:
                if (isRed) {
                    robot.controller.moveToPointPrecise(new Vector2D(-35.25, -61.75), Math.toRadians(270));
                } else {
                    robot.controller.moveToPointPrecise(new Vector2D(35.25, 61.75), Math.toRadians(90));
                }
                break;
        }
    }

    public void dropMacro() {
        switch (dropState) {
            case DROP_1:
                // robot.lift.liftLevel(1);
                robot.lift.liftAuto();
                dropState = ShootState.DROP_2;
                dropTimer.reset();

                break;
            case DROP_2:
                if (dropTimer.milliseconds() > 100) {
                    //       robot.intake.extendBasic();
                    //       robot.wrist.wristOut();
                    dropTimer.reset();
                    dropState = ShootState.PLACE_1;
                }
                break;
            case DROP_NO_PIXEL:
                // robot.lift.liftLevel(1);
                robot.lift.liftAuto();
                dropState = ShootState.DROP_NO2;
                shootCounter = 1;
                dropTimer.reset();

                break;
            case DROP_NO2:
                if (dropTimer.milliseconds() > 100) {
                    //              robot.intake.extendBasic();
                    //              robot.wrist.wristOut();
                    dropTimer.reset();
                    dropState = ShootState.PLACE_1;
                }
                break;
            case PLACE_AGAIN:
                robot.lift.liftAuto();
                dropTimer.reset();
                dropState = ShootState.PLACE_1;
                break;
            case PLACE_1:
                if (dropTimer.milliseconds() > 300) {
                    //     robot.wrist.turnerAuto(robot.getOrientation());
                    //robot.wrist.placePixel();
                    dropTimer.reset();
                    dropState = ShootState.SENSOR_DISTANCE;
                }

                break;
            case SENSOR_DISTANCE:
                if (((prox > 8 && robot.intake.extendo.getPosition() < 1))) {
                    //.68
                    //    System.out.println("14423 prox firsttime "+prox);
                    //    System.out.println("14423 firsttime servo "+robot.lift.extendo.getPosition());
                    //                    robot.lift.placeAuto();
                    //robot.lift.extendoPosition +=.01;//.015
                    //  robot.intake.extendertargetposition = .55;//TODO get proper distance*/
                    //robot.lift.extendToPos();
                } else {
                    dropTimer.reset();
                    //    System.out.println("14423 prox done "+prox);
                    //    System.out.println("14423 done extendo "+robot.lift.extendoPosition);
                    //    System.out.println("14423 done servo "+robot.lift.extendo.getPosition());
                    //  robot.lift.extendoPosition += 0.3;
                    dropState = ShootState.PLACE_2;
                }

                break;
            case PLACE_2:
                if (dropTimer.milliseconds() > 200) {
                    //robot.wrist.placePixel();

                    if (shootCounter > 0) {
                        //            robot.wrist.openDropper();
                        dropTimer.reset();
                        dropState = ShootState.PLACE_3;
                    } else {
                        //          robot.wrist.oneDropper();
                        //    System.out.println("14423 oneDropper");
                        dropTimer.reset();
                        dropState = ShootState.LIFTOUT;
                    }
                }
                break;
            case LIFTOUT:
                if (dropTimer.milliseconds() > 350) {
                    // System.out.println("14423 oneDropper close");
                    //       robot.wrist.closeDropper();

                    //   robot.wrist.turnerAuto(robot.getOrientation());

                    dropState = ShootState.RETRACT;
                    dropTimer.reset();
                }
                break;
            case RETRACT:
                if (dropTimer.milliseconds() > 200) {
                    //   robot.wrist.turnerAuto(robot.getOrientation());
                    //   robot.lift.extendoPosition -= 0.5;
                    // robot.lift.extendTOBoard(robot.getProx());
                    robot.lift.liftLevel(2);
                    //            robot.lift.returnFast();
                    dropState = ShootState.DROP_RETRACT;
                    dropTimer.reset();
                    shootCounter++;
                    //  robot.update();
                }
                break;
            case DROP_RETRACT:
                if (dropTimer.milliseconds() > 500) {
                    //            robot.lift.extendTOBoardAuto(prox);
                    //robot.wrist.turnerAuto(robot.getOrientation());
                    //  System.out.println("14423 prox " + prox);
                    //   System.out.println("14423 extendo " + robot.lift.extendo.getPosition());
                    if (prox > 20 || dropTimer.milliseconds() > 1500) {
                        dropState = ShootState.DROP_WAIT;
                    }
                }
                break;
            case DROP_WAIT:
                //         robot.wrist.turnerAuto(robot.getOrientation());
                robot.lift.liftAuto();
                break;
            case PLACE_3:
                if (dropTimer.milliseconds() > 400) {
                    //robot.wrist.dropPixel();

                    robot.lift.liftLevel(3);

                    dropTimer.reset();
                    dropState = ShootState.SOMERETRACT;
                }
                break;
            case SOMERETRACT:
                if (dropTimer.milliseconds() > 400) {
                    //robot.wrist.dropPixel();

                    //            robot.lift.returnFast();
                    //             robot.wrist.closeDropperReturn();

                    dropTimer.reset();
                    //dropState= DropState.SOMERETRACT;
                    dropState = ShootState.TURNRETURN;
                }

                break;
            case TURNRETURN:
                if (dropTimer.milliseconds() > 300) {
                    //             robot.wrist.turnerReturn();
                    dropTimer.reset();
                    dropState = ShootState.PLACE_4;
                }
                break;
            case PLACE_4:
                if (dropTimer.milliseconds() > 500) {
                    //             robot.lift.extendMin();
                    //robot.wrist.pickUp();
                    dropTimer.reset();
                    dropState = ShootState.WRISTIN;
                }
                break;
            case WRISTIN:
                if (dropTimer.milliseconds() > 400) {
                    //             robot.wrist.wristIn();
                    //robot.wrist.pickUp();
                    dropTimer.reset();
                    dropState = ShootState.PLACE_5;
                }
                break;
            case PLACE_5:
                if (dropTimer.milliseconds() > 500) {
                    robot.lift.setLiftPos(0);

                    dropState = ShootState.IDLE;
                }
                break;
            case INTAKE_OUT:
                //         robot.intake.fourBarOut();
                dropTimer.reset();
                dropState = ShootState.SPIT;
                break;
            case SPIT:
                if (dropTimer.milliseconds() > 1000) {
                    robot.intake.intakeSpit();
                    dropTimer.reset();
                    dropState = ShootState.INTAKE_IN;
                }
                break;
            case INTAKE_IN:
                if (dropTimer.milliseconds() > 2000) {
                    robot.intake.intakeOff();
                    dropState = ShootState.IDLE;
                }
            case IDLE:
                break;
        }
    }
/*
    public void updatePoseUltrasonic() {
        robot.ultrasonicLocalizer.update(Math.toRadians(270) - robot.getOrientation());
        //need to negate x because our OTOS flips x and so we flip it somewhere else in the code
        robot.setVector(robot.ultrasonicLocalizer.getPoseEstimate().x, robot.ultrasonicLocalizer.getPoseEstimate().y);
    }
*/
    private double calculateExtension() {
        if (robot.limelightResult != null) {
            double distance = 12.2 / Math.tan(Math.toRadians(17.8 + robot.limelightResult.getTy())) - 7.5; //6.5
            //   return (0.0263838*distance)+0.266513;
            return (0.02097 * distance) + 0.266513; //AXON
        } else {
            return .5;
        }
    }
}
//we leave this in in case we ever want to manually drive in auto
/* public void drive(){
        robot.controller.updateLocalizer(robot.getOrientation());
        Vector2D gamepadVector = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        Vector2D linearVelocityVector = gamepadVector.rotate(robot.getOrientation() - (robot.getAngularVelocity() * 0.1));
        telemetry.addData("Orientation", Math.toDegrees(robot.getOrientation()));
        telemetry.addData("Correction", robot.getAngularVelocity());
        robot.controller.setLinearVelocityVector(linearVelocityVector.x, linearVelocityVector.y);
        robot.controller.setAngularVelocity(-gamepad1.right_stick_x / 5);
        if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) >0.1 || Math.abs(gamepad1.right_stick_x) > 0.1 || gamepad1.right_bumper) {
            angle = linearVelocityVector.angle();
            robot.controller.update();
        }
        else{
            robot.controller.zeroPower(angle);
        }
    } */
