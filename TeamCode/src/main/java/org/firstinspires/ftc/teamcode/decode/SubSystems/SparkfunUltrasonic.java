package org.firstinspires.ftc.teamcode.decode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

@Config
@I2cDeviceType
@DeviceProperties(name = "Sparkfun Ultrasonic", description = "Sparkfun Ultrasonic", xmlTag = "SPARKFUN_ULTRASONIC")
public class SparkfunUltrasonic extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private ElapsedTime ultrasonicTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double dist = 0;
    public static double WAIT = 25;

    /*
// Available I2C addresses of the Qwiic Ultrasonic
const uint8_t kQwiicUltrasonicDefaultAddress = 0x2F;

// Firmware versions. The later hardware version is v10 and so the "latest" here
// refers to that. The previous version is randomnly given the value v01.
const uint8_t kQwiicUltrasonicFWLatest = 0x10;
const uint8_t kQwiicUltrasonicFWOld = 0x01;

// These addresses are the min and max (respectively) of valid I2C addresses that can
// be used for the newest revision of the Qwiic Ultrasonic sensor.
const uint8_t kQwiicUltrasonicI2CAddressMin = 0x08;
const uint8_t kQwiicUltrasonicI2CAddressMax = 0x77;

// Available I2C addresses of the older SparkFun Qwiic Ultrasonic Sensor.
const uint8_t kQwiicUltrasonicMinAddress = 0x20;
const uint8_t kQwiicUltrasonicMaxAddress = 0x2F;

     */

    private enum Commands {
        READ_COMMAND(0x01),
        ADDRESS_CHANGE_COMMAND(0x04);

        int bVal;

        Commands(int bVal) {
            this.bVal = bVal;
        }
    }

    public void setNewAddress1() {
        byte[] address = new byte[1];
        address[0] = (byte) 0x08;
        writeI2C(Commands.ADDRESS_CHANGE_COMMAND, address);
    }

    public void setNewAddress2() {
        byte[] address = new byte[1];
        address[0] = (byte) 0x09;
        writeI2C(Commands.ADDRESS_CHANGE_COMMAND, address);
    }

    public double readDistance() {
        int bytesRead;
        int numBytes = 2;
        byte[] rawData = new byte[2];
        if (ultrasonicTimer.milliseconds() > WAIT) {
            rawData = readI2C(Commands.READ_COMMAND, numBytes);
            //   System.out.println("RD0 " + rawData[0]);
            //  System.out.println("RD1 " + rawData[1]);
            dist = ((rawData[0] & 0xFF) << 8) | (rawData[1] & 0xFF);
            //  System.out.println("dist " + dist);
            ultrasonicTimer.reset();
        }
        return convertMmToInches(dist);
        //   return convertMmToInches(readShort(Commands.READ_COMMAND));
    }

    private double convertMmToInches(double mm) {
        return mm / 25.4;
    }

    private void writeI2C(Commands cmd, byte[] data) {
        deviceClient.write(cmd.bVal, data, I2cWaitControl.WRITTEN); //WRITTEN
    }

    private byte[] readI2C(Commands cmd, int len) {
        return deviceClient.read(cmd.bVal, len);
    }

    /* protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

     */

    protected short readShort(Commands cmd) {
        return TypeConversion.byteArrayToShort(deviceClient.read(cmd.bVal, 2));
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        ultrasonicTimer.startTime();
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Sparkfun Ultrasonic Sensor";
    }

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2F);
    private static final I2cAddr ADDRESS_I2C_NEW1 = I2cAddr.create7bit(0x08);
    private static final I2cAddr ADDRESS_I2C_NEW2 = I2cAddr.create7bit(0x09);

    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
            Commands.READ_COMMAND.bVal,
            2,
            I2cDeviceSynch.ReadMode.REPEAT
        );
        deviceClient.setReadWindow(readWindow);
    }

    public SparkfunUltrasonic(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        //this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.NONE);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}
