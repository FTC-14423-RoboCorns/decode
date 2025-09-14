package org.firstinspires.ftc.teamcode.decode.SubSystems;

/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/



import org.firstinspires.ftc.teamcode.decode.DecodeRobot;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;


public class DatalogWrapper
{
    Datalog datalog;
       DecodeRobot robot;

    public DatalogWrapper(DecodeRobot robot){
        this.robot =robot;
        String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
        datalog = new Datalog("datalog" + timestamp);

    }

    public void datalogWrite() {

        datalog.SparkX.set(robot.controller.getController().combinedLocalizer.getSparkVector().x );
        datalog.SparkY.set(robot.controller.getController().combinedLocalizer.getSparkVector().y );
        datalog.UltrasonicX.set(robot.controller.getController().combinedLocalizer.getUltrasonicVector().x );
        datalog.UltrasonicY.set(robot.controller.getController().combinedLocalizer.getUltrasonicVector().y );
        datalog.Angle.set(Math.toDegrees(robot.getOrientation()));
        datalog.LocalizerMode.set(String.valueOf(robot.controller.getController().combinedLocalizer.poseMode));
        datalog.Trigger.set(robot.controller.getController().getSparkVector().y<-48);
        datalog.writeLine();


        }

        /*
         * The datalog is automatically closed and flushed to disk after
         * the OpMode ends - no need to do that manually :')
         */


    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField SparkX = new Datalogger.GenericField("SparkX");
        public Datalogger.GenericField SparkY  = new Datalogger.GenericField("SparkY");
        public Datalogger.GenericField UltrasonicX          = new Datalogger.GenericField("UltrasonicX");
        public Datalogger.GenericField UltrasonicY        = new Datalogger.GenericField("UltrasonicY");
        public Datalogger.GenericField Angle        = new Datalogger.GenericField("Angle");
        public Datalogger.GenericField LocalizerMode        = new Datalogger.GenericField("LocalizerMode");
        public Datalogger.GenericField Trigger        = new Datalogger.GenericField("Trigger");
      //  public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
       // public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            LocalizerMode,
                            SparkX,
                            SparkY,
                            UltrasonicX,
                            UltrasonicY,
                            Angle,
                            Trigger
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }

}

