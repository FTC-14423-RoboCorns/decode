package org.firstinspires.ftc.teamcode.decode;


//import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.Vector2D;


/*
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
*/
public class CombinedLocalizer {
    private Vector2D currentVector = new Vector2D(0,0);
    private Vector2D sparkVector= new Vector2D(0,0);
    private Vector2D aprilTagVector= new Vector2D(0,0);
    private Vector2D ultrasonicVector= new Vector2D(0,0);

    private DecodeRobot robot;
  // private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  public enum PoseMode {
      SPARK,
      ULTRASONIC,
      BASKETULTRA,
      APRILTAG,
      COMBINED,
      FORCEULTRA;
  }
  public PoseMode poseMode =PoseMode.SPARK;

    public CombinedLocalizer( DecodeRobot robot){
        this.robot=robot;
    }


    public Vector2D getSparkVector(){
        return sparkVector;

    }
    public Vector2D getUltrasonicVector(){
        return ultrasonicVector;

    }

    public Vector2D getAprilTagVector(){
        return aprilTagVector;

    }

    public Vector2D getCurrentVector(){
        return currentVector;

    }
    public static double convertMetersToInches(double meters) { return meters * 39.3701; }
    public void localizerUpdate(){
        /*robot.controller.getController().updateSparkPose();
        robot.readLimelight();
        if (robot.limelightResult !=null) {
            aprilTagVector = new Vector2D(convertMetersToInches(robot.limelightResult.getBotpose_MT2().getPosition().x), -convertMetersToInches(robot.limelightResult.getBotpose_MT2().getPosition().y));
        }

         */
        if (poseMode==PoseMode.APRILTAG){
            robot.readLimelight();
            aprilTagVector=new Vector2D(convertMetersToInches(robot.limelightResult.getBotpose_MT2().getPosition().x),-convertMetersToInches(robot.limelightResult.getBotpose_MT2().getPosition().y));
            //add code to read apriltag
        }
       robot.controller.getController().updateSparkPose();
       sparkVector=robot.controller.getController().getSparkVector();
        ultrasonicVector = new Vector2D(0,0);
        double diff;
       if (poseMode==PoseMode.ULTRASONIC) {
           if (robot.ultrasonicLocalizer.getBlue()) {
           diff = Math.toRadians(270) - robot.getOrientation();}
           else {
               diff = Math.toRadians(90) - robot.getOrientation();}
           robot.ultrasonicLocalizer.update(diff);//may need isbluefor angle
           ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getPoseEstimate().x, robot.ultrasonicLocalizer.getPoseEstimate().y);
           if (Math.abs(diff) < Math.toRadians(20)) {
               robot.ultrasonicLocalizer.update(diff);//may need isbluefor angle
               ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getPoseEstimate().x, robot.ultrasonicLocalizer.getPoseEstimate().y);
               if (Math.abs(ultrasonicVector.subt(sparkVector).magnitude()) < 6) {
                   currentVector = ultrasonicVector;
               } else {
                   currentVector = sparkVector;
               }

           } else {
               currentVector=sparkVector;
           }
       } else if (poseMode==PoseMode.BASKETULTRA) {
           if (robot.ultrasonicLocalizer.getBlue()) {
               diff = Math.toRadians(90) - robot.getOrientation();}
           else {
               diff = Math.toRadians(270) - robot.getOrientation();}
           robot.ultrasonicLocalizer.updateDiagonal(diff);//may need isbluefor angle //TODO SET TO DIAGONALS
           ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getDiagonalPoseEstimate().x, robot.ultrasonicLocalizer.getDiagonalPoseEstimate().y);
           if (Math.abs(diff) < Math.toRadians(10)) {
               robot.ultrasonicLocalizer.updateDiagonal(diff);//may need isbluefor angle
               ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getDiagonalPoseEstimate().x, robot.ultrasonicLocalizer.getDiagonalPoseEstimate().y);
               currentVector = ultrasonicVector;
               }
           else {
               currentVector=sparkVector;
           }
       }
       else if (poseMode==PoseMode.FORCEULTRA) {
              if (robot.ultrasonicLocalizer.getBlue()) {
                   diff = Math.toRadians(270) - robot.getOrientation();}
               else {
                   diff = Math.toRadians(90) - robot.getOrientation();}

           robot.ultrasonicLocalizer.update(diff);//may need isbluefor angle
           ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getPoseEstimate().x, robot.ultrasonicLocalizer.getPoseEstimate().y);
           if (Math.abs(diff) < Math.toRadians(20)) {
               robot.ultrasonicLocalizer.update(diff);//may need isbluefor angle
               ultrasonicVector = new Vector2D(robot.ultrasonicLocalizer.getPoseEstimate().x, robot.ultrasonicLocalizer.getPoseEstimate().y);

                   currentVector = ultrasonicVector;

           } else {
               currentVector = ultrasonicVector;
               //currentVector=sparkVector;
           }
       }


           else
       {
            currentVector=sparkVector;
        }


    }



}