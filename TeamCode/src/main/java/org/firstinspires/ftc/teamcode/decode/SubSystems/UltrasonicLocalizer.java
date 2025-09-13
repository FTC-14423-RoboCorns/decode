package org.firstinspires.ftc.teamcode.decode.SubSystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Vector2D;
import org.jetbrains.annotations.NotNull;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class UltrasonicLocalizer {

    //public static double PARALLEL_X_BLUE = 0; // X is the up and down direction
    public static double PARALLEL_X_RED =  8;
    public static double PARALLEL_X_BLUE = 8;
    public static double PARALLEL_Y = 0; // Y is the strafe direction
    public double PARALLEL_X_FINAL;
    public static double X_MULTIPLIER =1.01694915;// 1.0059; //1.0163 was oldF
    public static double Y_MULTIPLIER =1.021276;// 1.0095;
    public double[] curPos={0,0};
    public double[] lastPos={0,0};
    public static double PERPENDICULAR_X = 6.34;
    public static double PERPENDICULAR_Y = 7.34;
    public int isBlue =-1; // 1 for blue, -1 for red
    public ElapsedTime trackingTimer =new ElapsedTime();
    public double oldTime,timeDiff=0;
    //public boolean red;
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public SparkfunUltrasonic parallelEncoder, perpendicularEncoder,oppositeEncoder;
    //private Localizer mecanumLocalizer;
    private Vector2D ourPose,diagonalPose;
    public double initialHeading;


    public static UltrasonicLocalizer makeUltrasonicTrackingLocalizer(HardwareMap hardwareMap, boolean isBlue) {
        if (!isBlue) {
            return new UltrasonicLocalizer(hardwareMap, PARALLEL_X_RED,isBlue);
        } else {
            return new UltrasonicLocalizer(hardwareMap, PARALLEL_X_BLUE,isBlue);
        }

    }

    public void setBlue(boolean Blue){
        if(!Blue){
            isBlue = 1;
        }
        else{
            isBlue = -1;
        }
    }

    public boolean getBlue(){
        if(isBlue == 1){
            return false; //yes, this is backward
        }
        else{
            return true;
        }
    }

    private UltrasonicLocalizer(HardwareMap hardwareMap, double PARALLEL_X,boolean Blue){
        // public TwoWheelTrackingLocalizer(HardwareMap hardwareMap){
//TODO: figure out how to adjust parallel_x for each side of robot


        PARALLEL_X_FINAL=PARALLEL_X;
        initSensors(hardwareMap,Blue);
        if(!Blue){
            isBlue = 1;//yes, this is backward. Blue is negative on the axis
        }
        else{
            isBlue = -1;
        }
        //this.mecanumLocalizer=mecanumLocalizer;

       // initialHeading=drive.getPoseEstimate().getHeading();
     //   trackingTimer.reset();


        //System.out.println("GYRO_angle "+());



    }

    /*@NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return ourPose;
    }
     */

    /*@Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        ourPose = pose2d;
    }
*/
    public void initSensors(HardwareMap hardwareMap,boolean blueside){

        perpendicularEncoder = hardwareMap.get(SparkfunUltrasonic.class, "rear");

        parallelEncoder = hardwareMap.get(SparkfunUltrasonic.class, "rightside"); //always use right side this year
        oppositeEncoder = hardwareMap.get(SparkfunUltrasonic.class, "leftside");
        /*  if (blueside) {
            parallelEncoder = hardwareMap.get(SparkfunUltrasonic.class, "rightside");
            //this.isBlue = -1;
        }
        else {
            parallelEncoder = hardwareMap.get(SparkfunUltrasonic.class, "leftside");
            //this.isBlue = 1;
        }

       */
    }


    @NotNull
    public Vector2D getPoseEstimate() {
        return ourPose;
    }

        public void setPoseEstimate(@NotNull Vector2D pose2d) {
        ourPose = pose2d;
    }

    public void update(double angle){

       // ourPose = new Vector2D(isBlue*(71-(Math.cos(angle)*parallelEncoder.readDistance())-PARALLEL_X_FINAL),isBlue*(71-(Math.cos(angle)*perpendicularEncoder.readDistance())-PERPENDICULAR_X));
        ourPose = new Vector2D(isBlue*(71-(parallelEncoder.readDistance())-PARALLEL_X_FINAL),isBlue*(71-(perpendicularEncoder.readDistance())-PERPENDICULAR_X));



    }

    public Vector2D getDiagonalPoseEstimate() {
        return diagonalPose;
    }

    public void setDiagonalPoseEstimate(@NotNull Vector2D pose2d) {
        diagonalPose = pose2d;
    }


    public void updateDiagonal (double angle) {
        //TODO: add code to set pose based on diagonal
        diagonalPose = new Vector2D(isBlue*(71-(Math.sin(angle)*oppositeEncoder.readDistance())-PARALLEL_X_FINAL),isBlue*(71-(Math.sin(Math.toRadians(90)-angle)*parallelEncoder.readDistance())-PARALLEL_X_FINAL));
    }

    //@Override
    /*public void update() {
        mecanumLocalizer.setPoseEstimate(ourPose);
        mecanumLocalizer.update();
        super.setPoseEstimate(ourPose);
        super.update();
        if ((poseEstimate.getX()>10)&&(Math.abs(poseEstimate.getY())>36)&&((poseEstimate.getHeading() >= Math.toDegrees(330)) || (poseEstimate.getHeading() <= Math.toDegrees(30))))
        {
            poseEstimate=super.getPoseEstimate();
        }
            else
        {
            poseEstimate=mecanumLocalizer.getPoseEstimate();
        }


        //ourPose=mecanumLocalizer.getPoseEstimate();
        //ourPose.plus(new Pose2d(0.1,0,0));

    }
     */


}