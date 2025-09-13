package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Vector2D;
@Config
public class CombinedPathFollower {
    public static double MAXRAM=.9;
    public Vector2D targetPos;
    public double remainingDistance;
    public double remainingAngle;
    private double lastAngle,lastDistance;
    public static double TSTEP=.01;
    private Vector2D resultant;
    private double targetAngle;
    double[][] points = {{0,0}, {0,0}, {0,0},{0,0}};
    double t = 0.0;
    private double xVelocity = 0;
    private boolean isDriving;
    private boolean isTurning = true;
    private  double yVelocity = 0;
    private double angularVelocity = 0;
    private double prevTime,prevDist = 0;
    private double currTime,currDist = 0;
    private double routeConstant = 0;
    double timeConstant,tRaw = 0.0;
    private IntoTheDeepRobot robot;
    private boolean precise=false;
    private boolean loose=false;
    public boolean abort=false;
    public int noMoveCycles=0;
    public ElapsedTime swerveTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private SwerveController controller;
    private PIDFController pidController = new PIDFController(0.04,0,0,0);//.05
    private PIDFController linePidController = new PIDFController(0.028,0.002,0.002,0);//.03
    private PIDFController ramPidController = new PIDFController(0.035 ,0.002,0.002,0);
    private PIDFController splinePidController = new PIDFController(0.028,0,0.002,0);//.05
    private PIDFController turnPidController = new PIDFController(0.17,0.03,0,0);//0.1//.13
    private PIDFController turnOnlyPidController = new PIDFController(0.3,0.01,0.0,0);//0.3
    private PIDFController turnOnlyFastPidController = new PIDFController(0.5,0.01,0,0);//0.1
    private boolean isFollowing=false;
    private boolean turnfast=false;
    private boolean ram=false;

    //private PIDFController pidController = new PIDFController(10,0,0,0,0,0);  
    enum PathState {
        LINE,
        TURN, CURVE
    }
    private PathState pathState =PathState.CURVE;

    public CombinedPathFollower(double xPos, double yPos, IntoTheDeepRobot robot){
        this.robot =robot;
        controller = new SwerveController(xPos, yPos,robot);
    }
    public void init(HardwareMap hardwareMap){
        controller.init(hardwareMap);
        //pidController = new PIDFController(10,0,0,0,0,0);
        //pidController.setIntegrationBounds(-1,1);
        //pidController.setTolerance(.5);
    }

    public void setPositions(double xPos, double yPos,double newAngle){
        controller.setPosition(xPos, yPos,newAngle);
    }
    public void setVector (double xPos, double yPos){
        controller.setVector(xPos, yPos);
    }


    public void zeroPower(){
        controller.initzeroPower(0);
    }

    public void resetTime(){
     //   swerveTimer.reset();
        currTime = 0;
        prevTime = 0;
        t = 0;
        currDist=0;
        currTime=0;
       // tRaw = 0;
    }

    public void updateLine(double robotAngle){
        controller.setIsAuto(true);
        resultant = targetPos.subt(controller.getPositionVector());
        if (resultant.magnitude()<1) {
           turnOnly(targetAngle);
        } else
        {
        //NOTE HOW resultant.angle IS NOW CALCULATED CORRECTLY IN NEXT LINE BECAUSE RESULTANT IS UPDATED HERE
        //YOU HAD HEADING AND MAGNITUDE REVERSED IN THE PARAMETERS (you had magnitude, heading), I FIXED IT
        Vector2D linearVelocityVector = Vector2D.fromHeadingAndMagnitude(resultant.angle(), calculateLinearSpeedPID());//calculateVelocity()- controller.getAngularVelocity() * 0.5
   /*     System.out.println("14423 targetpos" + targetPos);
        System.out.println("14423 currentpos" + controller.getPositionVector());

       System.out.println("14423 CurVector Position: " + controller.combinedLocalizer.getCurrentVector());
        System.out.println("14423 Spark Position: " + controller.combinedLocalizer.getSparkVector());
        System.out.println("14423 Ultra Position: " + controller.combinedLocalizer.getUltrasonicVector());
        //System.out.println("NewPosition: " + controller.getNewPositionVector());
        //System.out.println("ThirdPosition: " + controller.getThirdPositionVector());
        System.out.println("14423 linear velocity vector" + linearVelocityVector);
         System.out.println("14423 resultant"+resultant);
        System.out.println("14423 resultant angle" + Math.toDegrees(resultant.angle()));

    */
        linearVelocityVector = linearVelocityVector.rotate(robotAngle);//?????
    //    System.out.println("14423 rotated linear velocity vector" + linearVelocityVector);
    //    System.out.println("14423 rotated linear velocity angle" + Math.toDegrees(linearVelocityVector.angle()));
        xVelocity = linearVelocityVector.x;
        yVelocity = linearVelocityVector.y;
        if(isTurning){
            angularVelocity = calculateAngularVelocity(robotAngle);
         /*   if (xVelocity+yVelocity>.2) {//.1
            angularVelocity = calculateAngularVelocity(robotAngle);}
            else {
                angularVelocity=calculateTurnVelocity(robotAngle);
            }

          */

        }
        else{
            angularVelocity = 0;
        }

     //   System.out.println("14423 RobotAngle" + Math.toDegrees(robotAngle));
        //System.out.println("14423 AngularVelocity" + angularVelocity);
        if (atSetPoint(robotAngle)) {//atSetPoint
            isDriving = false;
            xVelocity = 0;
            yVelocity = 0;
            angularVelocity = 0;
            controller.zeroPower(0);
            linePidController.reset();
            turnPidController.reset();
            ramPidController.reset();
        } else {
            controller.setLinearVelocityVector(xVelocity, yVelocity);
            //TODO: Currently 0, need to incorporate turning to angle
            controller.setAngularVelocity(angularVelocity);
            controller.update();

            // System.out.println(resultant);
        }
        }
    }

    public void updateTurn(double robotAngle){
        controller.setIsAuto(true);
        //resultant = targetPos.subt(controller.getPositionVector());

        //NOTE HOW resultant.angle IS NOW CALCULATED CORRECTLY IN NEXT LINE BECAUSE RESULTANT IS UPDATED HERE
        //YOU HAD HEADING AND MAGNITUDE REVERSED IN THE PARAMETERS (you had magnitude, heading), I FIXED IT
       // Vector2D linearVelocityVector = Vector2D.fromHeadingAndMagnitude(resultant.angle(), calculateLinearSpeedPID());//calculateVelocity()- controller.getAngularVelocity() * 0.5
       // System.out.println("14423 targetpos" + targetPos);
       // System.out.println("14423 currentpos" + controller.getPositionVector());

       // System.out.println("14423 Position: " + controller.getPositionVector());
        //System.out.println("NewPosition: " + controller.getNewPositionVector());
        //System.out.println("ThirdPosition: " + controller.getThirdPositionVector());
       // System.out.println("14423 linear velocity vector" + linearVelocityVector);
       // System.out.println("14423 resultant"+resultant);
       // System.out.println("14423 resultant angle" + Math.toDegrees(resultant.angle()));
       // linearVelocityVector = linearVelocityVector.rotate(robotAngle);//?????
      //  System.out.println("14423 rotated linear velocity vector" + linearVelocityVector);
      //  System.out.println("14423 rotated linear velocity angle" + Math.toDegrees(linearVelocityVector.angle()));
       // xVelocity = linearVelocityVector.x;
       // yVelocity = linearVelocityVector.y;
        if(isTurning){
            xVelocity=0;
            yVelocity=0;
            angularVelocity = calculateTurnVelocity(robotAngle);
         /*   if (xVelocity<.01&&yVelocity<.01)
                angularVelocity=Range.clip(angularVelocity,.04,1);
            }

          */
        }
        else{
            angularVelocity = 0;
            xVelocity=0;
            yVelocity=0;
        }

        //System.out.println("14423 RobotAngle" + Math.toDegrees(robotAngle));
        //System.out.println("14423 AngularVelocity" + angularVelocity);
        if (isAtTurnSetPoint(robotAngle)) {
            isDriving = false;
            isTurning=false;
            xVelocity = 0;
            yVelocity = 0;
            turnfast=false;
            angularVelocity = 0;
            controller.zeroPower(0);
           // linePidController.reset();
            turnOnlyPidController.reset();
            turnOnlyFastPidController.reset();
            controller.update();
        } else {
            controller.setLinearVelocityVector(xVelocity, yVelocity);
            //TODO: Currently 0, need to incorporate turning to angle
            controller.setAngularVelocity(angularVelocity);
            controller.update();

            // System.out.println(resultant);
        }
    }



    public void updateCurve(double robotAngle){
      //  System.out.println("Position: " + controller.getPositionVector());
      //  System.out.println("14423 final target: " + new Vector2D(points[0][0], points[0][1]));
        //System.out.println("ThirdPosition: " + controller.getThirdPositionVector());
        controller.setIsAuto(true);// need to add this to handle isAligned
        if (isAtSetPoint(robotAngle)) {
            isDriving=false;
            controller.zeroPower(0);
            pidController.reset();
            splinePidController.reset();
            turnPidController.reset();
        } else{
            updateTime();
            Vector2D linearVelocityVector = calculateLinearVelocity().rotate(robotAngle);

            xVelocity = linearVelocityVector.x;
            yVelocity = linearVelocityVector.y;
           // angularVelocity = calculateCurveAngularVelocity(robotAngle);
           angularVelocity = calculateAngularVelocity(robotAngle);

           /* if (xVelocity+yVelocity>.1) {
                angularVelocity = calculateAngularVelocity(robotAngle);}
            else {
                angularVelocity=calculateTurnVelocity(robotAngle);
            }

            */

            if (t < 1 && !isAtSetPoint(robotAngle)) {

                controller.setLinearVelocityVector(xVelocity, yVelocity);
                //TODO: Currently 0, need to incorporate turning to angle
                controller.setAngularVelocity(angularVelocity);
                controller.update();
                //System.out.println("time " + t);
                // System.out.println("RawTime " + tRaw);
                //System.out.println("CurveVelocity: " + linearVelocityVector.magnitude() + " Angle: " + Math.toDegrees(linearVelocityVector.angle()));
                //System.out.println("Position: " + controller.getPositionVector());
                //System.out.println("Target: "+calculateTarget(t));
                //System.out.println("RobotAngle " + Math.toDegrees(robotAngle));

            } else if (t >= 1 && !isAtSetPoint(robotAngle)) {
                if (precise) {
                    moveToPointPrecise(new Vector2D(points[0][0], points[0][1]), targetAngle);
                } else {
                    moveToPoint(new Vector2D(points[0][0], points[0][1]), targetAngle);
                }

                /*
                Vector2D errorVector = new Vector2D(points[0][0], points[0][1]).subt(controller.getPositionVector());
                double speed = 0;
                speed = pidController.calculate(errorVector.magnitude());
                linearVelocityVector = Vector2D.fromHeadingAndMagnitude(errorVector.angle(), speed);

                xVelocity = linearVelocityVector.x;
                yVelocity = linearVelocityVector.y;
                angularVelocity = calculateCurveAngularVelocity(robotAngle);
              //  angularVelocity = calculateAngularVelocity(robotAngle);

                controller.setLinearVelocityVector(xVelocity, yVelocity);
                //TODO: Currently 0, need to incorporate turning to angle
                controller.setAngularVelocity(angularVelocity);

                 */
                controller.update();

                //System.out.println("time " + t);
                // System.out.println("RawTime " + tRaw);
                //System.out.println("CurveVelocity: " + linearVelocityVector.magnitude() + " Angle: " + Math.toDegrees(linearVelocityVector.angle()));
                //System.out.println("RobotAngle " + Math.toDegrees(robotAngle));

            }
        }

    }

    public Vector2D calculateTarget(double t){
        double xTarget = (points[0][0] * t * t * t) - (3 * points[1][0] * t * t * t) + (3 * points[2][0] * t * t * t) - (points[3][0] * t * t * t) + (3 * points[1][0] * t * t) + (3 * points[3][0] * t * t) - (6 * points[2][0] * t * t) + (3*points[2][0] * t) - (3*points[3][0] * t) + points[3][0];
        double yTarget = (points[0][1] * t * t * t) - (3 * points[1][1] * t * t * t) + (3 * points[2][1] * t * t * t) - (points[3][1] * t * t * t) + (3 * points[1][1] * t * t) + (3 * points[3][1] * t * t) - (6 * points[2][1] * t * t) + (3*points[2][1] * t) - (3*points[3][1] * t) + points[3][1];
        //System.out.println("targetPosition: " + new Vector2D(xTarget, yTarget).subt(controller.getPositionVector()));
        return new Vector2D(xTarget, yTarget);
    }

    public boolean atSetPoint(double robotAngle) {
        double error =  targetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        lastAngle=remainingAngle;
        lastDistance=remainingDistance;
        remainingAngle=error;
        remainingDistance=resultant.magnitude();

     //   System.out.println("14423 precise "+precise);
       if (precise) {
           return Math.abs(resultant.x) < .5 &&Math.abs(resultant.y)<.5 && Math.abs(error) < Math.toRadians(2);
       } else if (loose) {
           return resultant.magnitude() < 3.5 && Math.abs(error) < Math.toRadians(5);//1.5
       } else
       {
           return resultant.magnitude() < 1  && Math.abs(error) < Math.toRadians(1.5 );//1.5
       }
    }


    public boolean isAtSetPoint(double robotAngle){
        double error =  targetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        lastAngle=remainingAngle;
        lastDistance=remainingDistance;
        remainingAngle=Math.abs(calculateError(targetAngle, robotAngle));
        remainingDistance=controller.getPositionVector().subt(new Vector2D(points[0][0], points[0][1])).magnitude();
        //TODO: Refactor to make faster
       // System.out.println("14423 off magnitude" + controller.getPositionVector().subt(new Vector2D(points[0][0], points[0][1])).magnitude());
       // System.out.println("14423 off angle "+Math.toDegrees(Math.abs(calculateError(targetAngle, robotAngle))));
        if (loose) {
            if(controller.getPositionVector().subt(new Vector2D(points[0][0], points[0][1])).magnitude() < 3.5 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(5)){//1 and 2
                //  if(resultant.magnitude() < 1 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(2)){//1.5 and 3
                return true;
            }
        } else if (precise) {
            if (controller.getPositionVector().subt(new Vector2D(points[0][0], points[0][1])).magnitude() < .5 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(1)) {//1 and 2
                //  if(resultant.magnitude() < 1 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(2)){//1.5 and 3
                return true;
            }
        }  else {
        if(controller.getPositionVector().subt(new Vector2D(points[0][0], points[0][1])).magnitude() < 1.5 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(3)) {//1 and 2
            //  if(resultant.magnitude() < 1 && Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(2)){//1.5 and 3
            return true;
        }
        }
        return false;
    }

    public boolean isAtTurnSetPoint(double robotAngle){
        lastAngle=remainingAngle;
        lastDistance=remainingDistance;
        remainingAngle=Math.abs(calculateError(targetAngle, robotAngle));
        remainingDistance=0;
        if (turnfast) {
            if (Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(4)) {
                return true;
            }
        } else {
            if(Math.abs(calculateError(targetAngle, robotAngle)) < Math.toRadians(1.5)){
                return true;
            }
        }
        return false;
    }

    public boolean isNotMoving() {
        return (Math.abs(lastAngle-remainingAngle)<.01&&Math.abs(lastDistance-remainingDistance)<.1);
    }

    public boolean isDoneRoute(double robotAngle){
        if(t >= 1 || isAtSetPoint(robotAngle)){
            return true;
        }
        return false;
    }


    public void updateTime(){///THIS IS YOUR BUG! If the pods are not aligned you are resetting your time!!!!!
        //TODO: update time by fixed and distance
        ///We can do this with an absolute time, but not a realtime!!!! And just pause, not reset!!!!
        do {
        currDist=calculateResultantVector(t+TSTEP).magnitude();
        prevDist=calculateResultantVector(t).magnitude();
        if(prevDist>currDist) {
            t+=TSTEP;
        }
        }
        while (prevDist>currDist);
        /*if (t==0) {
            isFollowing=false;
            prevDist=calculateResultantVector(0).magnitude();
            t+=TSTEP;

        } else {
            t+=TSTEP;
            currDist=calculateResultantVector(t).magnitude();
            while()

            if ((currDist - prevDist)<=0) {//this is not quite right. Need to figure out how to make sure we are in front of robot

                prevDist=currDist;

            }
        }*/
/*
        if(!controller.getIsAligned()){
            currTime = 0;
            prevTime = 0;
            swerveTimer.reset();
        }
        else{
            currTime = swerveTimer.milliseconds();
            t += (currTime-prevTime)/(1000 * timeConstant);
            tRaw += currTime-prevTime;
            if (t > 1){
                t = 1;
            }
        }
        prevTime = currTime;


 */

    }

    public double calculateDistance(double lowerBound){
        double sum = 0;
        for(double i = lowerBound; i<1; i+=0.01){
            sum += Math.sqrt(Math.pow(spline(i)[0],2) + Math.pow(spline(i)[1],2));
        }
        return sum * 0.01;
    }

    public Vector2D calculateResultantVector(double t){
        double xTarget = (points[0][0] * t * t * t) - (3 * points[1][0] * t * t * t) + (3 * points[2][0] * t * t * t) - (points[3][0] * t * t * t) + (3 * points[1][0] * t * t) + (3 * points[3][0] * t * t) - (6 * points[2][0] * t * t) + (3*points[2][0] * t) - (3*points[3][0] * t) + points[3][0];
        double yTarget = (points[0][1] * t * t * t) - (3 * points[1][1] * t * t * t) + (3 * points[2][1] * t * t * t) - (points[3][1] * t * t * t) + (3 * points[1][1] * t * t) + (3 * points[3][1] * t * t) - (6 * points[2][1] * t * t) + (3*points[2][1] * t) - (3*points[3][1] * t) + points[3][1];
        //System.out.println("targetPosition: " + new Vector2D(xTarget, yTarget).subt(controller.getPositionVector()));
        return new Vector2D(xTarget, yTarget).subt(controller.getPositionVector());
    }



    public Vector2D scaledLinearVector(){
        double speed = 0;
        Vector2D resultant = calculateResultantVector(t);
        pidController.reset();
        pidController.setSetPoint(0);
        double rawPID =Math.abs(pidController.calculate(resultant.magnitude()));
        speed = Math.signum(rawPID)* Range.clip(Math.abs(rawPID),0, .6);
        //System.out.println("199 ScaledLinearVector: " + resultant);
        //System.out.println("199 ScaledLinearVectorAngle: " + Math.toDegrees(resultant.angle()));
        //System.out.println("199 ScaledLinearVectorSpeed: " + speed);
        return Vector2D.fromHeadingAndMagnitude(resultant.angle(), speed);
    }

    public Vector2D calculateLinearVelocity(){
        Vector2D resultant = scaledLinearVector();
        resultant.add(calculateSplineVelocity());
        return resultant;
    }

    public Vector2D calculateSplineVelocity(){
        //timeConstant = calculateDistance(0) / routeConstant;
        timeConstant = calculateDistance(0) / 36;//23.75;

        double[] direction = spline(t);
        Vector2D linearVelocityVector = new Vector2D(direction[0], direction[1]);
        double linearSpeed = 0;
        double remaining=calculateDistance(t);
        double rawPID =Math.abs(splinePidController.calculate(remaining));
        if (ram){
            linearSpeed = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .3  , .6);
        } else {
            linearSpeed = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .1, .6);
        }
        double rad=radius(t);
      //  System.out.println("14423 Velocity Angle " + Math.toDegrees(linearVelocityVector.angle()));
       // System.out.println("14423 Radius " + rad);
        //System.out.println("DistanceLeft " + calculateDistance(t));
      //DEPRECATED for PID
        /*
        if(calculateDistance(t) < 20){//15
            //linearSpeed = calculateDistance(t) / 50;
            linearSpeed = calculateDistance(t) / 60 + 0.1;//50
        }
        else{
            linearSpeed = 0.35;//.6
        }

         */
        linearVelocityVector = Vector2D.fromHeadingAndMagnitude(linearVelocityVector.angle(), linearSpeed);
        //System.out.println("199 SplineVelocity: " + linearVelocityVector);
        return linearVelocityVector;
    }

    public double[]  spline(double t){//double[]
        double dx = (3 * points[0][0] * t * t) - (9 * points[1][0] * t * t) + (9 * points[2][0] * t * t) - (3 * points[3][0] * t * t) + (6 * points[1][0] * t) + (6 * points[3][0] * t) - (12 * points[2][0] * t) + (3*points[2][0]) - (3*points[3][0]);
        double dy = (3 * points[0][1] * t * t) - (9 * points[1][1] * t * t) + (9 * points[2][1] * t * t) - (3 * points[3][1] * t * t) + (6 * points[1][1] * t) + (6 * points[3][1] * t) - (12 * points[2][1] * t) + (3*points[2][1]) - (3*points[3][1]);
        return new double[]{dx,dy};
    }
    public double[]  second(double t){//double[]
        double ddx = (6 * points[0][0] * t ) - (18 * points[1][0] * t ) + (18 * points[2][0] * t ) - (6 * points[3][0] * t ) + (6 * points[1][0] ) + (6 * points[3][0] ) - (12 * points[2][0] ) ;
        double ddy = (6 * points[0][1] * t) - (18 * points[1][1] * t ) + (18 * points[2][1] * t ) - (6 * points[3][1] * t ) + (6 * points[1][1] ) + (6 * points[3][1] ) - (12 * points[2][1] ) ;
        return new double[]{ddx,ddy};
    }
public double radius(double t){
        double[] d=spline(t);
        double[] dd=second(t);
        double rad= (Math.pow(Math.pow(d[0],2)+Math.pow(d[1],2),(3/2)))/Math.abs((d[0]*dd[1])-(d[1]*dd[0]));
        return rad;

}


    public void update(double robotAngle){
      //  System.out.println("14423 Position: " + controller.getPositionVector());
      //  System.out.println("14423 RobotAngle" + Math.toDegrees(robotAngle));
      //  System.out.println("14423 rear ultra "+robot.ultrasonicLocalizer.perpendicularEncoder.readDistance() );
        if (abort) {
            isDriving = false;
            isTurning=false;
            xVelocity = 0;
            yVelocity = 0;
            angularVelocity = 0;
           // controller.zeroPower(0);
             linePidController.reset();
            turnPidController.reset();
            ramPidController.reset();

            controller.update();
            abort=false;

        } else {
      // controller.updateLocalizer(robotAngle);

        switch (pathState) {
            //TODO: We will need to have one robot class and one controller class to be robost keep working with this and I can refactor at some point

            case LINE:
            //THE NEXT LINE COMES FROM calculateVelocity SEE COMMENT THERE FOR WHY
                updateLine(robotAngle);
            break;
            case CURVE:
                updateCurve(robotAngle);
            break;
            case TURN:
                updateTurn(robotAngle);
                break;
        }
            if (isNotMoving()){
                noMoveCycles++;

            }
        }
    }

    public double calculateTurnVelocity(double robotAngle){
        double error =  targetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        // System.out.println("14423 AngularError" + Math.toDegrees(error));
        if (turnfast) {
            if (Math.abs(error) < Math.toRadians(3)) {

                turnOnlyFastPidController.reset();
                return 0;
            } else {
                return turnPIDTurn(error);
            }
        } else {
            if (Math.abs(error) < Math.toRadians(1)) {

                turnOnlyPidController.reset();
                return 0;
            } else {
                return turnPIDTurn(error);
            }
        }
       /* DEPRECATING turnPID. All turnPIDcontroller
       if (pathState==PathState.LINE) {
        return turnPID(error);
        } else
        {return turnPIDCurve(error);}*/
    }



    public double calculateAngularVelocity(double robotAngle){
        double error =  targetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
       // System.out.println("14423 AngularError" + Math.toDegrees(error));
        if (Math.abs(error)<Math.toRadians(2)) {//2
            turnPidController.reset();
            return 0;
        } else {
            return turnPIDCurve(error);
        }
       /* DEPRECATING turnPID. All turnPIDcontroller
       if (pathState==PathState.LINE) {
        return turnPID(error);
        } else
        {return turnPIDCurve(error);}*/
    }

    public double calculateCurveAngularVelocity(double robotAngle){
        double error = calculateError(targetAngle, robotAngle) * t;
        System.out.println("AngleError " + Math.toDegrees(error));
        double speed = turnPidController.calculate(error);
        return speed;
    }

    public double calculateError(double currTargetAngle, double robotAngle){
        double error =  currTargetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        return error;
    }

    public void setCurveRam(double[][] points, double targetAngle, double routeConstant){
        loose=false;
        ram=true;
        precise=false;
        pathState=PathState.CURVE;
        isDriving=true;
        isFollowing=true;
        resetTime();
        this.points = points;
        this.targetAngle = targetAngle;
        t = 0;
        resetTime();
        pidController.reset();
        pidController.setSetPoint(0);
        splinePidController.reset();
        splinePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        this.routeConstant = routeConstant;
        noMoveCycles=0;
    }

    public void setCurve(double[][] points, double targetAngle, double routeConstant){
        loose=false;
        ram=false;
        precise=false;
        pathState=PathState.CURVE;
        isDriving=true;
        isFollowing=true;
        resetTime();
        this.points = points;
        this.targetAngle = targetAngle;
        t = 0;
        resetTime();
        pidController.reset();
        pidController.setSetPoint(0);
        splinePidController.reset();
        splinePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        this.routeConstant = routeConstant;
        noMoveCycles=0;
    }

    public void setCurvePrecise(double[][] points, double targetAngle, double routeConstant){
        loose=false;
        ram=false;
        precise=true;
        pathState=PathState.CURVE;
        isDriving=true;
        isFollowing=true;
        resetTime();
        this.points = points;
        this.targetAngle = targetAngle;
        t = 0;
        resetTime();
        pidController.reset();
        pidController.setSetPoint(0);
        splinePidController.reset();
        splinePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        this.routeConstant = routeConstant;
        noMoveCycles=0;
    }


    public void setCurveLoose(double[][] points, double targetAngle, double routeConstant){
        loose=true;
        pathState=PathState.CURVE;
        isDriving=true;
        isFollowing=true;
        resetTime();
        this.points = points;
        this.targetAngle = targetAngle;
        t = 0;
        resetTime();
        pidController.reset();
        pidController.setSetPoint(0);
        splinePidController.reset();
        splinePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        this.routeConstant = routeConstant;
    }

    public double turnPID(double error){
        if(error > Math.toRadians(40)){//CHANGED ALL TO PLUS
            return 0.05;//-.05
        }
        else if (error > Math.toRadians(10)){//TODO: consider making this larger
            return 0.03;//-.03
        }
        else if(error > Math.toRadians(4)){//4
            return 0.02;//-.02
        }
        else if(error < Math.toRadians(-40)){ //CHANGED ALL TO MINUS
            return -0.05;//.05
        }
        else if (error < Math.toRadians(-10)){//TODO: consider making this larger
            return -0.03;//.03
        }
        else if (error < Math.toRadians(-4)){//-4
            return -0.02;//.02
        }
        else{
            return 0;
        }
    }

    public double turnPIDCurve(double error){
        /*if(error > Math.toRadians(40)){//CHANGED ALL TO PLUS
            return -0.07;//-.05
        }
        else if (error > Math.toRadians(10)){
            return -0.05;//-.03
        }
        else if(error > Math.toRadians(4)){//4
            return -0.04;//-.02
        }
        else if(error < Math.toRadians(-40)){ //CHANGED ALL TO MINUS
            return 0.07;//.05
        }
        else if (error < Math.toRadians(-10)){
            return 0.05;//.03
        }
        else if (error < Math.toRadians(-4)){//-4
            return 0.04;//.02
        }
        else{
            return 0;
        }*/
        double rawPID=turnPidController.calculate(error);
        //double servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),.04,.09);
        double servoPower= Math.signum(rawPID)* Range.clip(Math.abs(rawPID),.04,.09);
        return -servoPower;
    }

    public double turnPIDTurn(double error){
        /*if(error > Math.toRadians(40)){//CHANGED ALL TO PLUS
            return -0.07;//-.05
        }
        else if (error > Math.toRadians(10)){
            return -0.05;//-.03
        }
        else if(error > Math.toRadians(4)){//4
            return -0.04;//-.02
        }
        else if(error < Math.toRadians(-40)){ //CHANGED ALL TO MINUS
            return 0.07;//.05
        }
        else if (error < Math.toRadians(-10)){
            return 0.05;//.03
        }
        else if (error < Math.toRadians(-4)){//-4
            return 0.04;//.02
        }
        else{
            return 0;
        }*/
        if (turnfast){
            double rawPID = turnOnlyFastPidController.calculate(error);
            double servoPower = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .04, .9);
            return -servoPower;
        } else {
            double rawPID = turnOnlyPidController.calculate(error);
            double servoPower = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .04, .8);//.04
            return -servoPower;
        }

    }


    public double calculateLinearSpeedPID(){

        if (resultant.magnitude()<.5)
        {   linePidController.reset();
            ramPidController.reset();
            return 0;
        }
        isTurning=true;
        //Going to try always turning
     /*   if (resultant.magnitude() > 20){//48
            isTurning = false;

        } else{
            isTurning = true;

        }*/

        double speed = 0;
        if (ram) {
            double rawPID = Math.abs(ramPidController.calculate(resultant.magnitude()));
            speed = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .2, MAXRAM);//.35
            return speed;
        } else {
            double rawPID = Math.abs(linePidController.calculate(resultant.magnitude()));
            speed = Math.signum(rawPID) * Range.clip(Math.abs(rawPID), .1, .7);//.5
            return speed;
        }
    }

    //calculateVelocity DEPRECATED for PID
    public double calculateVelocity(){//MOVE SCALING HERE
       //I MOVED THIS UP TO UPDATE, BECAUSE YOU NEED TO UPDATE RESULTANT NOT JUST FOR VELOCITY BUT ALSO FOR ANGLE
        // resultant = targetPos.subt(controller.getPositionVector());
       //System.out.println("14423 position error "+resultant.magnitude());
        if (resultant.magnitude()<.5)
        {return 0;}
        else if (resultant.magnitude() > 20){//48
            isTurning = false;
            return 0.3;//.3
        } else{
            isTurning = true;
            return (resultant.magnitude() /90 ) + .1;//100+.1
        }
      /*  else if(resultant.magnitude() > 8){
            return resultant.magnitude() / 40;//80
        }
       else{
            return 0.1;
        }

       */
    }

    public double getXPos(){
        return controller.getXPos();
    }
    public double getYPos(){
        return controller.getYPos();
    }

    public void moveToPointPrecise(Vector2D targetPos, double targetAngle){
        precise=true;
        loose=false;
        ram=false;
        pathState=PathState.LINE;
        isDriving = true;
        this.targetPos = targetPos;
        linePidController.reset();
        linePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        resultant = targetPos.subt(controller.getPositionVector());
        this.targetAngle = targetAngle;
        noMoveCycles=0;

    }

    public void moveToPointLoose(Vector2D targetPos, double targetAngle){
        loose=true;
        precise=false;
        ram=false;

        pathState=PathState.LINE;
        isDriving = true;
        this.targetPos = targetPos;
        linePidController.reset();
        linePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        resultant = targetPos.subt(controller.getPositionVector());
        this.targetAngle = targetAngle;
        noMoveCycles=0;
    }

    public void moveToPointRam(Vector2D targetPos, double targetAngle){
        ram=true;
        loose=false;
        precise=false;
        pathState=PathState.LINE;
        isDriving = true;
        this.targetPos = targetPos;
        ramPidController.reset();
        ramPidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        resultant = targetPos.subt(controller.getPositionVector());
        this.targetAngle = targetAngle;
        noMoveCycles=0;
    }

    public void moveToPoint(Vector2D targetPos, double targetAngle){
       precise=false;
       loose=false;
       ram=false;
        pathState=PathState.LINE;
        isDriving = true;
        this.targetPos = targetPos;
        linePidController.reset();
        linePidController.setSetPoint(0);
        turnPidController.reset();
        turnPidController.setSetPoint(0);
        resultant = targetPos.subt(controller.getPositionVector());
        this.targetAngle = targetAngle;
        noMoveCycles=0;
    }

    public void turnOnly(double targetAngle){

            pathState=PathState.TURN;
            isDriving = true;
            isTurning = true;
            turnfast=false;
            turnOnlyPidController.reset();
            turnOnlyPidController.setSetPoint(0);
            //resultant = targetPos.subt(controller.getPositionVector());
            this.targetAngle = targetAngle;
            noMoveCycles=0;

    }

    public void turnOnlyFast(double targetAngle){

        pathState=PathState.TURN;
        isDriving = true;
        isTurning = true;
        turnfast=true;
        turnOnlyFastPidController.reset();
        turnOnlyFastPidController.setSetPoint(0);
        //resultant = targetPos.subt(controller.getPositionVector());
        this.targetAngle = targetAngle;
        noMoveCycles=0;

    }


    public void turn(double targetAngle, double robotAngle){
        controller.setLinearVelocityVector(0, 0);
        double error =  targetAngle - robotAngle;
        if(error > Math.toRadians(180)){
            error -= Math.toRadians(360);
        }
        else if(error < -Math.toRadians(180)){
            error += Math.toRadians(360);
        }
        if(error > Math.toRadians(40)){
            controller.setAngularVelocity(0.2);
        }
        else if (error > Math.toRadians(2)){
            controller.setAngularVelocity(0.05);
        }
        else if (error < Math.toRadians(-40)){
            controller.setAngularVelocity(-0.2);
        }
        else if (error < Math.toRadians(-2)){
            controller.setAngularVelocity(-0.05);
        }
        else{
            controller.setAngularVelocity(0);
            controller.zeroPower(Math.toRadians(90));//robotAngle;
        }
        controller.update();
       // System.out.println(Math.toDegrees(error));
        if(Math.abs(error) < Math.toRadians(2)){
            isDriving = false;
        }
        else{
            isDriving = true;
        }
    }

    public boolean isDriving(){
        return isDriving;
    }



    public SwerveController getController(){
        return controller;
    }
}
