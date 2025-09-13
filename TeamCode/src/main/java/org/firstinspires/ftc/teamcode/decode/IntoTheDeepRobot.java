package org.firstinspires.ftc.teamcode.decode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.Data;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.DatalogWrapper;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.DatalogWrapperElectricity;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.OurQwiicLEDStick;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.SparkfunOdo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.SubSystems.UltrasonicLocalizer;

import java.util.List;

public class IntoTheDeepRobot {
   // public AutoSwervePathFollower controller;
    //public CurveBasedSwervePathFollower controller;
    public CombinedPathFollower controller;
    public IMU imu;
    public Arm arm = new Arm();
    public Lift lift = new Lift();
    public Intake intake = new Intake();
  //  public Servo purple;
    public DistanceSensor prox;
    public SparkfunOdo sparkODO = new SparkfunOdo();
    public UltrasonicLocalizer ultrasonicLocalizer;
    public List<LynxModule> allHubs;
    public VoltageSensor battery;
    public DatalogWrapper datalogWrapper;
    public DatalogWrapperElectricity datalogWrapperElectricity;
    //public Husky huskylens = new Husky();
  //  public Frontcam frontcam;
    public int[] rainbow = new int[] { Color.rgb(148, 0, 211), Color.rgb(75, 0, 130), Color.rgb(0, 0, 255),
            Color.rgb(0, 255, 0), Color.rgb(255, 0, 0), Color.rgb(255, 255, 0), Color.parseColor("purple"),
            Color.parseColor("teal"), Color.parseColor("silver"), Color.rgb(0, 0, 0) };
    public int[] white = new int[] { Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"),
            Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"),
            Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white") };
    public int[] pixel = new int[] { Color.parseColor("purple"), Color.parseColor("silver"), Color.parseColor("purple"), Color.parseColor("silver"), Color.parseColor("purple"), Color.parseColor("silver"), Color.parseColor("purple"), Color.parseColor("silver"), Color.parseColor("purple"), Color.parseColor("silver") };
   public int[] ledColor =new int[] { Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"),
            Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white"),
            Color.parseColor("white"), Color.parseColor("white"), Color.parseColor("white") };



//public int[] ledColor;

    public OurQwiicLEDStick  ledStrip;
    YawPitchRollAngles orientation;
    public double imuAngle=0;
    public double outtakeProx;
    private double angle;
    public Limelight3A limelight;
    public LLResult limelightResult;
    public IntoTheDeepRobot(double xPos, double yPos, double angle){



                //  controller = new AutoSwervePathFollower(xPos, yPos);
        //controller = new CurveBasedSwervePathFollower(xPos, yPos);

        this.angle = angle;
    }
    public void init(HardwareMap hardwareMap, boolean isReset,Telemetry telem){
        allHubs=hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
      //  purple=  hardwareMap.get(Servo.class, "Purple");
      //  prox = hardwareMap.get(DistanceSensor.class, "prox");
      //  purpleOpen();

      //  wrist.init(hardwareMap);
      //  datalogWrapper=new DatalogWrapper(this);
      //  ledColor= new int[] {Color.RED, Color.YELLOW, Color.RED, Color.YELLOW, Color.RED,
       //         Color.YELLOW, Color.RED, Color.YELLOW, Color.RED, Color.YELLOW};
        datalogWrapperElectricity=new DatalogWrapperElectricity(this);
        battery = hardwareMap.voltageSensor.get("Control Hub");
        lift.init(hardwareMap);
        intake.init(hardwareMap);
        arm.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(3);
        limelight.start();
        ledinit(hardwareMap);
        //huskylens.init(hardwareMap);
      //  webcam =new Webcam(hardwareMap, telem);
      //  frontcam=new Frontcam(hardwareMap, telem);
/*            imu = hardwareMap.get(IMU.class, "imumain");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;*/
       // imu = hardwareMap.get(IMU.class, "imu");
       // RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
       // RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        sparkODO.init(hardwareMap);
       // RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        ultrasonicLocalizer= UltrasonicLocalizer.makeUltrasonicTrackingLocalizer(hardwareMap, Data.getBlue());
      //      imu.initialize(new IMU.Parameters(orientationOnRobot));
            if(isReset){
        //    imu.resetYaw();
            }
            else{
              //  angle += Math.toRadians(90);//90
            }

        controller = new CombinedPathFollower(0, 0,this);
        controller.init(hardwareMap);

    }

   //TODO:Add reading only when needed
    public void readi2c(){
      //  orientation = imu.getRobotYawPitchRollAngles();
        //System.out.println("14423IMU" + Math.toDegrees((orientation.getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)));
        //imuAngle= orientation.getYaw(AngleUnit.RADIANS);


       // outtakeProx=getProx();
       // if (outtakeProx>30){outtakeProx=30;}
      /*OLD intake
        if (intake.isIntakeOn) { //only read intake prox if we are intaking
            intake.getIntakeProx();
        }

       */
      //  intake.getExtendProx();
        readLimelight(); //TODO: read only when needed

    }
//TODO:deal with when there is no result
public void readLimelight() {
    limelight.updateRobotOrientation(Math.toDegrees(getOrientation()));
    LLResult result = limelight.getLatestResult();
    if (result != null) {
        if (result.isValid()) {
            limelightResult = result;

            //Pose3D botpose = result.getBotpose();
            //telemetry.addData("tx", result.getTx());
            //telemetry.addData("ty", result.getTy());
            //telemetry.addData("Botpose", botpose.toString());
        }
    }
}

    public void ledinit(HardwareMap hardwareMap) {
        ledStrip = hardwareMap.get(OurQwiicLEDStick.class, "ledstick");
        ledStrip.setBrightness(4);
       // ledStrip.setColors(rainbow);
        ledStrip.setColors(white);
      //  ledStrip.setColor(Color.parseColor("red"));
        //ledStrip.setColor(5, Color.parseColor("blue"));
    }

    public double getProx(){
        return ((DistanceSensor) prox).getDistance(DistanceUnit.CM) ;
    }
    public void ledstop() {
        ledStrip.turnAllOff();
    }

    public void update(){
     //   wrist.update();
        lift.update();
        intake.update();
        arm.update();
    }
    public void setPose(double xPos, double yPos, double newAngle){
        controller.setPositions(xPos, yPos,newAngle);
       // resetYaw();
     //   angle = newAngle;
        //controller.getController().localizer.setFTCLibPose(xPos, yPos,newAngle,getRawGyro());
    }

    public void setVector(double xPos, double yPos){
        controller.setVector(xPos, yPos);
        // resetYaw();
        //   angle = newAngle;
        //controller.getController().localizer.setFTCLibPose(xPos, yPos,newAngle,getRawGyro());
    }

  /*  public void setPoseAprilSpecific(int tag){
        AprilTagPoseFtc pose;
        pose =webcam.getPose(tag);
        if (pose != null){
            VectorF tagPos = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(tag+1).fieldPosition;
            //VectorF tagY = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(tag+1).fieldPosition;
            double xoffset=Math.sin(getOrientation()) * 9;
            double yoffset=Math.cos(getOrientation()) * 9;
            controller.setPositions(tagPos.get(0) - (pose.y)+xoffset, -tagPos.get(1) - pose.x+yoffset,getOrientation());
            webcam.foundTag=0;
        } else {
        setPoseAprilAny();

        }




        //controller.getController().localizer.setFTCLibPose(xPos, yPos,newAngle,getRawGyro());
    }

   */




    public double getOrientation(){
      //  orientation = imu.getRobotYawPitchRollAngles();
        //System.out.println("14423IMU" + Math.toDegrees((orientation.getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)));
        return (((imuAngle + Math.toRadians(360)) % Math.toRadians(360)) + angle) % Math.toRadians(360);
       // return ((imuAngle ) + angle) % Math.toRadians(360);
    }

    public double getRawGyro(){
       // orientation = imu.getRobotYawPitchRollAngles();
        //System.out.println("14423IMU" + Math.toDegrees((orientation.getYaw(AngleUnit.RADIANS) + Math.toRadians(360)) % Math.toRadians(360)));
        return (((imuAngle + Math.toRadians(360)) % Math.toRadians(360))) % Math.toRadians(360);
    }


    public void resetYaw(){
     //   imu.resetYaw();
        sparkODO.setpose(sparkODO.getPose().x,sparkODO.getPose().y,0);
        angle = Math.toRadians(0);
    }
    public void resetYawManual(){
       // imu.resetYaw();
        sparkODO.configureOtos();
        sparkODO.setpose(sparkODO.getPose().x,sparkODO.getPose().y,0);
        angle = Math.toRadians(0);//270
    }

    public void ledSide(boolean blue){
        if (blue) {
        ledColor[0]=Color.parseColor("blue");
            ledColor[1]=Color.parseColor("blue");}
    else {
        ledColor[0]=Color.parseColor("red");
            ledColor[1]=Color.parseColor("red");
    }
    }

    public void ledTarget(boolean blue){
        if (lift.targetSpecimen()) {
        if (blue) {
            ledColor[2]=Color.parseColor("black");
            ledColor[3]=Color.parseColor("blue");
            ledColor[4]=Color.parseColor("blue");}
        else {
            ledColor[2]=Color.parseColor("black");
            ledColor[3]=Color.parseColor("red");
            ledColor[4]=Color.parseColor("red");
        } } else {
            ledColor[2]=Color.parseColor("black");
            ledColor[3]=Color.parseColor("yellow");
            ledColor[4]=Color.parseColor("yellow");
        }
        }

    public void ledPlace(){
        ledColor[5]=Color.parseColor("black");
        ledColor[6]=Color.parseColor("purple");
        ledColor[7]=Color.parseColor("purple");
        ledColor[8]=Color.parseColor("purple");
        ledColor[9]=Color.parseColor("purple");
    }

    public void ledIntake(){
        ledColor[5]=Color.parseColor("black");
        ledColor[6]=Color.parseColor("green");
        ledColor[7]=Color.parseColor("green");
        ledColor[8]=Color.parseColor("green");
        ledColor[9]=Color.parseColor("green");
    }

    public void ledPickup(){
        ledColor[5]=Color.parseColor("black");
        ledColor[6]=Color.parseColor("teal");
        ledColor[7]=Color.parseColor("teal");
        ledColor[8]=Color.parseColor("teal");
        ledColor[9]=Color.parseColor("teal");
    }

    public void ledEmpty(){
        ledColor[5]=Color.parseColor("black");
        ledColor[6]=Color.parseColor("red");
        ledColor[7]=Color.parseColor("red");
        ledColor[8]=Color.parseColor("red");
        ledColor[9]=Color.parseColor("red");
    }


}
