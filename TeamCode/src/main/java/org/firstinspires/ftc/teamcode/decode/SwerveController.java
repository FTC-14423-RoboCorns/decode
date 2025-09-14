package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Vector2D;

public class SwerveController {
    private Vector2D linearVelocityVector = new Vector2D(0,0);
    private double angularVelocity = 0;
    public SwervePod frontLeft;
    public SwervePod frontRight;
    public SwervePod backLeft;
    public SwervePod backRight;
    private DecodeRobot robot;
    public boolean isAligned = false;
    public static double LENGTH = 11.9/2;//10
    public static double WIDTH = 12.1/2;//9.875
    public static double ODOSCALE=1.0;//54/47.75;
    private Vector2D poseVector=new Vector2D(0,0);
    private boolean isAuto;
    public SwerveLocalizer localizer;
    private SwervePod[] pods;
    public CombinedLocalizer combinedLocalizer;
    double[] wheelPowers = new double[4];
    public SwerveController(double xPos, double yPos, DecodeRobot robot){
        this.robot=robot;
        localizer = new SwerveLocalizer(xPos, yPos,robot);
        combinedLocalizer=new CombinedLocalizer(robot);
        setLocalizerMode(CombinedLocalizer.PoseMode.SPARK);

    }


    public void init(HardwareMap hardwareMap){
        frontLeft = new SwervePod(hardwareMap, "LeftFrontEncoder", "LeftFrontServo", "LeftFront", false,Math.toRadians(137.5  ));
        frontRight = new SwervePod(hardwareMap, "RightFrontEncoder", "RightFrontServo", "RightFront", false,Math.toRadians(148));//248
        backLeft = new SwervePod(hardwareMap, "LeftRearEncoder", "LeftRearServo", "LeftRear", false,Math.toRadians(155));
        backRight = new SwervePod(hardwareMap, "RightRearEncoder", "RightRearServo", "RightRear", false ,Math.toRadians(154.5));//156.5
/*
        frontLeft.turnTo(0);
        frontRight.turnTo(0);
        backLeft.turnTo(0);
        backRight.turnTo(0);
*/
       // frontRight.reversemotor();
       // backRight.reversemotor();
        pods = new SwervePod[]{frontLeft, frontRight,backLeft,backRight};
        //localizer = new SwerveLocalizer(0,0);
    }
    public void setLinearVelocityVector(double x, double y){
        linearVelocityVector.setX(x);
        linearVelocityVector.setY(y);
    }
    public void setAngularVelocity(double angularVelocity){
        this.angularVelocity = angularVelocity;
    }

    public boolean isPodError(){
        for(SwervePod pod:pods){
            if(pod.getError() > Math.toRadians(35)){
                return true;
            }
        }
        return false;
    }

    public void turnMotorsOff(double velocity){
        frontRight.setMotorPower(0.1);
        frontLeft.setMotorPower(0.1);
        backRight.setMotorPower(0.1);
        backLeft.setMotorPower(0.1);
    }
    public void zeroPower(double angle){
        frontRight.setMotorPower(0);
        frontLeft.setMotorPower(0);
        backRight.setMotorPower(0);
        backLeft.setMotorPower(0);

        frontLeft.turnTo(angle);
        frontRight.turnTo(angle);
        backLeft.turnTo(angle);
        backRight.turnTo(angle);

      // System.out.println("FrontRight: " + Math.toDegrees(frontRight.readAngle()));
     //  System.out.println("FrontLeft: " +Math.toDegrees(frontLeft.readAngle()));
     //  System.out.println("BackRight: " +Math.toDegrees(backRight.readAngle()));
     //   System.out.println("BackLeft: " +Math.toDegrees(backLeft.readAngle()));
    }

    public void initzeroPower(double angle){
        frontRight.setMotorPower(0);
        frontLeft.setMotorPower(0);
        backRight.setMotorPower(0);
        backLeft.setMotorPower(0);
        frontLeft.initTurnTo(angle);
        frontRight.initTurnTo(angle);
        backLeft.initTurnTo(angle);
        backRight.initTurnTo(angle);
    }

    public void setPosition(double xPos, double yPos, double newAngle){
        localizer.xPos = xPos;
        localizer.yPos = yPos;
        localizer.newX=xPos;
        localizer.newY=yPos;
        robot.sparkODO.setpose(-xPos,yPos,newAngle);
      //  localizer.resetOdometry(new Pose2d(xPos,yPos,new Rotation2d(newAngle)),new Rotation2d(robot.getRawGyro()));

    }

    public void setVector (double xPos, double yPos){
        localizer.xPos = xPos;
        localizer.yPos = yPos;
        localizer.newX=xPos;
        localizer.newY=yPos;
        robot.sparkODO.setpose(-xPos,yPos,robot.sparkODO.getPose().h);
        //  localizer.resetOdometry(new Pose2d(xPos,yPos,new Rotation2d(newAngle)),new Rotation2d(robot.getRawGyro()));

    }

    public double getXPos(){
        return -robot.sparkODO.getPose().x;
     //   return localizer.getxPos();
    }
    public double getYPos(){
        return robot.sparkODO.getPose().y;
        //return localizer.getyPos();
    }

    public double getAngularVelocity(){
        return angularVelocity;
    }

    public void setIsAuto(boolean isAuto){
        this.isAuto = isAuto;
    }

    public Vector2D getPositionVector(){
        //return poseVector;
        return combinedLocalizer.getCurrentVector();
        //return new Vector2D(-robot.sparkODO.getPose().x*ODOSCALE,robot.sparkODO.getPose().y*ODOSCALE);
       // return localizer.getVector();


    }
    public Vector2D getSparkVector(){
        return poseVector;
        //return new Vector2D(-robot.sparkODO.getPose().x*ODOSCALE,robot.sparkODO.getPose().y*ODOSCALE);
        // return localizer.getVector();


    }
    public Vector2D getNewPositionVector(){
        return localizer.getNewVector();
    }
  /*  public Vector2D getThirdPositionVector(){
        return localizer.getThirdVector();
    }

   */
    public boolean getIsAligned(){
        return isAligned;
    }
    public void update(){
       // System.out.println("Position: " + getPositionVector());
        if(isPodError()){
            isAligned = false;
        }
        else{
            isAligned = true;
        }

      /*  Vector2D leftFrontVector = new Vector2D(1,1);
        Vector2D rightFrontVector = new Vector2D(-1,1);
        Vector2D leftRearVector = new Vector2D(1,-1);
        Vector2D rightRearVector = new Vector2D(-1,-1);


       */
        /* V2
        Vector2D leftFrontVector = new Vector2D(-1,-1);
        Vector2D rightFrontVector = new Vector2D(1,1);
        Vector2D leftRearVector = new Vector2D(-1,1);
        Vector2D rightRearVector = new Vector2D(1,-1);


         */

        Vector2D leftFrontVector = new Vector2D(-WIDTH/LENGTH,-1);
        Vector2D rightFrontVector = new Vector2D(WIDTH/LENGTH,-1);
        Vector2D leftRearVector =new Vector2D(-WIDTH/LENGTH,1);
        Vector2D rightRearVector = new Vector2D(WIDTH/LENGTH,1);

        /*Vector2d(vx - omega * y, vy + omega * x),
                Vector2d(vx - omega * y, vy - omega * x),
                Vector2d(vx + omega * y, vy - omega * x),
                Vector2d(vx + omega * y, vy + omega * x)*/
        leftFrontVector.mult(angularVelocity);
        rightFrontVector.mult(angularVelocity);
        leftRearVector.mult(angularVelocity);
        rightRearVector.mult(angularVelocity);

        leftFrontVector.add(linearVelocityVector);
        rightFrontVector.add(linearVelocityVector);
        leftRearVector.add(linearVelocityVector);
        rightRearVector.add(linearVelocityVector);
if (!isAligned && isAuto) {
    wheelPowers[0] = leftFrontVector.magnitude()*.1;
    wheelPowers[1] = rightFrontVector.magnitude()*.1;
    wheelPowers[2] = leftRearVector.magnitude()*.1;
    wheelPowers[3] = rightRearVector.magnitude()*.1;
} else if ( isAuto) {
    wheelPowers[0] = leftFrontVector.magnitude();
    wheelPowers[1] = rightFrontVector.magnitude();
    wheelPowers[2] = leftRearVector.magnitude();
    wheelPowers[3] = rightRearVector.magnitude();
} else  {
    wheelPowers[0] = leftFrontVector.magnitude();
    wheelPowers[1] = rightFrontVector.magnitude();
    wheelPowers[2] = leftRearVector.magnitude();
    wheelPowers[3] = rightRearVector.magnitude();
}
        double maxPow = 0;
        for(double powers : wheelPowers){
            if (Math.abs(powers) > maxPow){
                maxPow = Math.abs(powers);
            }
        }
        if(maxPow > 1){
            for(int i = 0; i<4; i++){
                wheelPowers[i] /= maxPow;
            }
        }


        frontLeft.turnTo(leftFrontVector.angle());
        frontRight.turnTo(rightFrontVector.angle());
        backLeft.turnTo(leftRearVector.angle());
        backRight.turnTo(rightRearVector.angle());

        if ( isAuto) {
            frontLeft.setMotorPower(wheelPowers[0]);
            frontRight.setMotorPower(wheelPowers[1]);
            backLeft.setMotorPower(wheelPowers[2]);
            backRight.setMotorPower(wheelPowers[3]);

        } else  {
            frontLeft.setMotorPower(wheelPowers[0]);//*.6
            frontRight.setMotorPower(wheelPowers[1]);//*.6
            backLeft.setMotorPower(wheelPowers[2]);//*.6
            backRight.setMotorPower(wheelPowers[3]);//*.6
          //  System.out.println("14423 Wheel Powers "+wheelPowers[0]+" "+wheelPowers[1]+" "+wheelPowers[2]+" "+wheelPowers[3] );
        }
 // System.out.println("14423 Wheel Powers "+wheelPowers[0]+" "+wheelPowers[1]+" "+wheelPowers[2]+" "+wheelPowers[3] );



    }

    public void updateSparkPose(){
        //don't need manual localizer, save time
        //localizer.localizerUpdate(pods, robotAngle);
        robot.sparkODO.readPose();
        poseVector=new Vector2D(-robot.sparkODO.getPose().x*ODOSCALE,robot.sparkODO.getPose().y*ODOSCALE);
        robot.imuAngle=robot.sparkODO.getPose().h;

    }

    public void updateLocalizer(double robotAngle){
        //don't need manual localizer, save time
        //localizer.localizerUpdate(pods, robotAngle);
        robot.sparkODO.readPose();
        poseVector=new Vector2D(-robot.sparkODO.getPose().x*ODOSCALE,robot.sparkODO.getPose().y*ODOSCALE);

        robot.imuAngle=robot.sparkODO.getPose().h;
      //  Data.setPose(poseVector);
      //  Data.setHeading(robot.getOrientation());
    }

    public void updateCombinedLocalizer(){
        //don't need manual localizer, save time
        //localizer.localizerUpdate(pods, robotAngle);
        combinedLocalizer.localizerUpdate();
    }

    public void setLocalizerMode(CombinedLocalizer.PoseMode poseMode){
        combinedLocalizer.poseMode=poseMode;
    }
}
