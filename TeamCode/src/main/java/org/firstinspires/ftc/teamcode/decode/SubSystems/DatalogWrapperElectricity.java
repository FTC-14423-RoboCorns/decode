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



import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.IntoTheDeepRobot;


public class DatalogWrapperElectricity
{
    Datalog datalog;
       IntoTheDeepRobot robot;

    public DatalogWrapperElectricity(IntoTheDeepRobot robot){
        this.robot =robot;
        datalog = new Datalog("datalog_03");

    }

    public void datalogWrite(String state) {

        datalog.State.set(state);
        datalog.AuxiliaryVoltage.set(robot.allHubs.get(0).getAuxiliaryVoltage(VoltageUnit.VOLTS));
        datalog.InputVoltage.set(robot.allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS));
        datalog.i2cBusCurrent.set(robot.allHubs.get(0).getI2cBusCurrent(CurrentUnit.AMPS));
        datalog.HubCurrent.set(robot.allHubs.get(0).getCurrent(CurrentUnit.AMPS));
        datalog.GpioBusCurrent.set(robot.allHubs.get(0).getGpioBusCurrent(CurrentUnit.AMPS));


        datalog.FrontLeft.set(robot.controller.getController().frontLeft.driveMotor.getCurrent(CurrentUnit.AMPS));
        datalog.BackLeft.set(robot.controller.getController().backLeft.driveMotor.getCurrent(CurrentUnit.AMPS));
        datalog.FrontRight.set(robot.controller.getController().frontRight.driveMotor.getCurrent(CurrentUnit.AMPS));
        datalog.BackRight.set(robot.controller.getController().backRight.driveMotor.getCurrent(CurrentUnit.AMPS));
        datalog.LiftCurrent.set(robot.lift.liftRear.getCurrent(CurrentUnit.AMPS));
        datalog.LiftPos.set(robot.lift.liftRear.getCurrentPosition());
       datalog.Battery.set(robot.battery.getVoltage());
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
        public Datalogger.GenericField State = new Datalogger.GenericField("State");
        public Datalogger.GenericField InputVoltage  = new Datalogger.GenericField("InputVoltage");
        public Datalogger.GenericField AuxiliaryVoltage  = new Datalogger.GenericField("AuxiliaryVoltage");
        public Datalogger.GenericField i2cBusCurrent = new Datalogger.GenericField("i2cBusCurrent");
        public Datalogger.GenericField HubCurrent = new Datalogger.GenericField("HubCurrent");
        public Datalogger.GenericField GpioBusCurrent = new Datalogger.GenericField("GpioCurrent");
        public Datalogger.GenericField LiftPos = new Datalogger.GenericField("LiftPos");
        public Datalogger.GenericField FrontLeft = new Datalogger.GenericField("FrontLeft");
        public Datalogger.GenericField BackLeft = new Datalogger.GenericField("BackLeft");
        public Datalogger.GenericField FrontRight = new Datalogger.GenericField("FrontRight");
        public Datalogger.GenericField BackRight = new Datalogger.GenericField("BackRight");
        public Datalogger.GenericField LiftCurrent = new Datalogger.GenericField("LiftCurrent");
      //  public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField Battery      = new Datalogger.GenericField("Battery");

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
                            State,
                            FrontLeft,
                            FrontRight,
                            BackLeft,
                            BackRight,

                            InputVoltage,
                            AuxiliaryVoltage,
                            i2cBusCurrent,
                            GpioBusCurrent,
                            HubCurrent,
                            LiftPos,
                            LiftCurrent
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

