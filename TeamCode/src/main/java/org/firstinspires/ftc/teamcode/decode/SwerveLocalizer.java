package org.firstinspires.ftc.teamcode.decode;


//import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.decode.SubSystems.SparkfunOdo;
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
public class SwerveLocalizer {
    private  SimpleMatrix m_forwardKinematics;
    private  SimpleMatrix m_inverseKinematics;
    double wheelDiameter = 2.8346;
    double ticksPerRotation = 336;//336
    double xPos,xDelta,newX;
    double yPos,yDelta,newY;
    double[] positions = new double[4];
    double[] angles = new double[4];
    double[] absAngles= new double[4];
    static int m_numModules=4;
    double[] lastEncReads = {0,0,0,0};
    double lastheading,angleChange;
    private double angle;
    private SparkfunOdo sparkODO;
    private SwervePod[] swervePods;
    private DecodeRobot robot;
    private SwerveModuleState[] moduleStates=new SwerveModuleState[4];
    public Pose2d m_pose;
double dx, dy,dtheta;
    Vector2D[] posVectors = new Vector2D[4];
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
  // private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveLocalizer(double xPos, double yPos, DecodeRobot robot){
        this.robot=robot;
        this.xPos = xPos;
        this.newX=xPos;
        this.yPos = yPos;
        this.newY=yPos;
        //Translation2d[] m_modules;
       /* Translation2d m_frontLeftLocation =
                new Translation2d(4.92, 4.92);
        Translation2d m_frontRightLocation =
                new Translation2d(-4.92, 4.92);
        Translation2d m_backLeftLocation =
                new Translation2d(4.92, -4.92);
        Translation2d m_backRightLocation =
                new Translation2d(-4.92, -4.92);
        Translation2d[] m_modules= new Translation2d[] {m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation};


        */
      /*JUST CHANGED
      Vector2D m_frontLeftLocation =
                new Vector2D(4.9375, 5);
        Vector2D m_frontRightLocation =
                new Vector2D(-4.9375, 5);
        Vector2D m_backLeftLocation =
                new Vector2D(4.9375, -5);
        Vector2D m_backRightLocation =
                new Vector2D(-4.9375, -5);

       */

        Vector2D m_frontLeftLocation =
                new Vector2D(-9.875/2, -5);
        Vector2D m_frontRightLocation =
                new Vector2D(9.875/2, -5);
        Vector2D m_backLeftLocation =
                new Vector2D(-9.875/2, 5);
        Vector2D m_backRightLocation =
                new Vector2D(9.875/2, 5);



        Vector2D[] m_modules= new Vector2D[] {m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation};


        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].y);
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].x);
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    //   initswerveodo(robot.getRawGyro());
    }


    private void initswerveodo(double gyroangle){
        Translation2d m_frontLeftLocation =
                new Translation2d(0.122, 0.122);
        Translation2d m_frontRightLocation =
                new Translation2d(-0.122, 0.122);
        Translation2d m_backLeftLocation =
                new Translation2d(0.122, -0.122);
        Translation2d m_backRightLocation =
                new Translation2d(-0.122, -0.122);
        m_kinematics = new SwerveDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );
        // this.swervePods=pods;
        for (int j = 0; j < 4; j++)
         {
            moduleStates[j]=new SwerveModuleState(0,Rotation2d.fromDegrees(0));
        }
        m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(gyroangle) );
        /*swerveDrivePoseEstimator =
                new SwerveDrivePoseEstimator(
                        m_kinematics,
                        Rotation2d.fromRadians(angle),
                        getModulePositions(angle),
                        new Pose2d(new Translation2d(0, 0),
                                Rotation2d.fromDegrees(0))); // x,y,heading in radians; Vision measurement std dev, higher=less weight


         */
    }




    /*
         var moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

       for (int i = 0; i < m_numModules; i++) {
         var module = moduleDeltas[i];
         moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
         moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
       }

       var chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
       return new Twist2d(
           chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
        */
    public void localizerUpdate(SwervePod[] pods, double robotAngle){
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        int i = 0;
        for(SwervePod pod : pods){
            positions[i] = pod.getEncoderPos();
            angles[i] = pod.readAngle();
            posVectors[i]  = Vector2D.fromHeadingAndMagnitude((angles[i] - robotAngle) % Math.toRadians(360), Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])));
            absAngles[i]=(angles[i] - robotAngle) % Math.toRadians(360);
            //System.out.println("position angle "+i+" " +Math.toDegrees(angles[i]));
            //System.out.println("position absangle "+i+" " +Math.toDegrees(absAngles[i]));
            //System.out.println("position raw prev "+i+" " +(tickToInches((lastEncReads[i]))));
            //System.out.println("position raw now"+i+" " +(tickToInches(positions[i])));
            //System.out.println("position raw diff"+i+" " +(tickToInches(positions[i]-lastEncReads[i])));
            //System.out.println("position "+i+" " +(tickToInches(Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])))));
            //System.out.println("position x"+i+" " +(tickToInches(Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])))) * Math.cos(absAngles[i]));
            //System.out.println("position y"+i+" " +(tickToInches(Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])))) * Math.sin(absAngles[i]));
            //System.out.println("position velocity"+i+" " +tickToInches(pod.getVelocity()));


           // var module = moduleDeltas[i];
           moduleDeltaMatrix.set(i * 2, 0, (tickToInches(Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])))) * Math.cos(absAngles[i]));
           moduleDeltaMatrix.set(i * 2 + 1, (tickToInches(Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])))) * Math.sin(absAngles[i]));
   //         moduleDeltaMatrix.set(i * 2, 0, (tickToInches(posVectors[i].x)));
   //         moduleDeltaMatrix.set(i * 2 + 1, (tickToInches(posVectors[i].y)));
        //    moduleStates[i].angle=new Rotation2d(absAngles[i]);
          //  moduleStates[i].speedMetersPerSecond=inchesToMeters(tickToInches(pod.getVelocity()));
            lastEncReads[i] = positions[i];
           // System.out.println("matrix " +moduleDeltaMatrix);
            i++;
        }

        SimpleMatrix chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
        dx=chassisDeltaVector.get(0, 0);
        dy=chassisDeltaVector.get(1, 0);
        // dtheta=chassisDeltaVector.get(2, 0);//decide whether to use gyro
        dtheta=Angle.normDelta(robotAngle-lastheading);
     //   Vector2D fullDelta=relativeOdometryUpdate(new Pose2d(xPos,yPos,robotAngle),new Pose2d(xDelta,yDelta,angleChange));
        Vector2D fullDelta=relativeOdometryUpdate(dx,dy,dtheta);
        newX+= fullDelta.x;
        newY+= fullDelta.y;

/*
        int i = 0;
        for(SwervePod pod : pods){
            positions[i] = pod.getEncoderPos();
            angles[i] = pod.readAngle();
            posVectors[i]  = Vector2D.fromHeadingAndMagnitude((angles[i] - robotAngle) % Math.toRadians(360), Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])));
            lastEncReads[i] = positions[i];
            i++;
        }

 */
        Vector2D resultant = new Vector2D(0,0);
        for(Vector2D posVector : posVectors){
            resultant.add(posVector);
        }
        angleChange=Angle.normDelta(robotAngle-lastheading);


         xDelta=tickToInches(resultant.x/4);
         yDelta=tickToInches(resultant.y/4);
         xPos += xDelta;
        yPos += yDelta;
      // Pose2d fullDelta=relativeOdometryUpdate(new Pose2d(xPos,yPos,Rotation2d.fromRadians(robotAngle)),new Pose2d(xDelta,yDelta,Rotation2d.fromRadians(angleChange));

      //  xPos += fullDelta.getX();
       // yPos += fullDelta.getY();
       //  xPos += fullDelta.x;
        // yPos += fullDelta.y;
      //  Rotation2d gyroAngle = new Rotation2d(robot.getRawGyro());
    //    m_pose = m_odometry.updateWithTime(System.currentTimeMillis()*1000,gyroAngle, moduleStates );
        lastheading=robotAngle;
      //  System.out.println("14423 Angle "+robotAngle);
       // System.out.println((angles[1] - robotAngle) % Math.toRadians(360));
       /*
        Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getHeading());
        // Update the pose
        m_pose = m_odometry.update
                (
                        gyroAngle, m_frontLeftModule.getState(),
                        m_frontRightModule.getState(),
                        m_backLeftModule.getState(), m_backRightModule.getState()
                );

         */

    }
/*
    public SwerveModulePosition[] getModulePositions(double robotAngle)
    {
        SwerveModulePosition[] modulePositions =
                new SwerveModulePosition[4];
        int i = 0;
        for (SwervePod pod : swervePods)
        {
            positions[i] = pod.getEncoderPos();
            angles[i] = pod.readAngle();
            posVectors[i]  = Vector2D.fromHeadingAndMagnitude((angles[i] - robotAngle) % Math.toRadians(360), Math.abs(Math.abs(positions[i]) - Math.abs(lastEncReads[i])));
            lastEncReads[i] = positions[i];
            modulePositions[i].angle=Rotation2d.fromRadians(angles[i]);
            modulePositions[i].distanceMeters= Units.inchesToMeters(tickToInches(positions[i]));
            i++
        }
        return modulePositions;
    }



 */


   public Vector2D relativeOdometryUpdate(double dx, double dy, double dtheta )  {

     //   double dtheta = robotPoseDelta.getRotation().getRadians();
     //  double dtheta = robotPoseDelta.getHeading();
        double sineTerm,cosTerm;
        if (Math.abs(dtheta) < 1E-9) {
            sineTerm=1.0 - 1.0 / 6.0 * dtheta * dtheta;//1.0 - dtheta * dtheta / 6.0;
            cosTerm=0.5 * dtheta;//dtheta / 2.0;
        } else {
            sineTerm= Math.sin(dtheta) / dtheta;
            cosTerm=(1 - Math.cos(dtheta)) / dtheta;
        }

        Vector2D fieldPositionDelta = new Vector2D(
                sineTerm * dx - cosTerm * dy,
                cosTerm * dx + sineTerm * dy
        );



//        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotated(fieldPose.getHeading()), robotPoseDelta.getHeading());
       // Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta, robotPoseDelta.getHeading());

        return fieldPositionDelta;

        /*new Pose2d(
                fieldPose.getX() + fieldPoseDelta.getX(),
                fieldPose.getY() + fieldPoseDelta.getY(),
                Angle.norm(fieldPose.getHeading() + fieldPoseDelta.getHeading()));
*/
    }





    public double tickToInches(double ticks){
        return (ticks/ticksPerRotation)*Math.PI*wheelDiameter;
    }
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public double getxPos(){
        return xPos;
    }
    public double getyPos(){

        return yPos;
    }
    public double getNewX(){
        return newX;
    }
    public double getNewY(){
        return newY;
    }

    public double getThirdX(){
        return metersToInches(m_pose.getX());
    }
    public double getThirdY(){
        return metersToInches(m_pose.getY());
    }

    public Vector2D getVector(){
        //System.out.println(xPos);
       // robot.setPoseAprilAny();
        return new Vector2D(xPos, yPos);
    }

    public Vector2D getNewVector(){
        //System.out.println(xPos);
        return new Vector2D(newX, newY);
    }

    public Vector2D getThirdVector(){
        //System.out.println(xPos);
        return new Vector2D(metersToInches(m_pose.getX()), metersToInches(m_pose.getY()));
    }



    public void resetOdometry(Pose2d pose,Rotation2d raw)
    {

        m_odometry.resetPosition(pose,raw);

    }



}