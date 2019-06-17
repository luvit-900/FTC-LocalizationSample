package com.terabytesrobotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RobotDefinitions;

/**
 * Created by Ronald on 10/6/2017.
 */

public class SBSIMU
{

   //IMU Values
    private double XAcceleration;
    private double YAcceleration;
    private double ZAcceleration;
    private double Yaw;
    private double Roll;
    private double Pitch;

    // The IMU sensor object
    BNO055IMU imu1;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    String IMU = "imu1";

    private SBSDataLogger dataLogger;

    private boolean auton;
    private String OpModeName = "null";
    private double OpmodeInitialOffset;
    private RobotDefinitions.AutonOpModeColor AutonOpModeColor;

    public SBSIMU(BNO055IMU imu1, boolean auton, String OpModeName, double opmodeInitialOffset, RobotDefinitions.AutonOpModeColor OpModeColor, SBSDataLogger dataLogger)
    {
        this.imu1 = imu1;
        this.auton = auton;
        this.OpModeName = OpModeName;
        this.OpmodeInitialOffset = opmodeInitialOffset;
        this.AutonOpModeColor = OpModeColor;
        this.dataLogger = dataLogger;
    }

    public void init()
    {
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();

        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = IMU +"_BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu1.initialize(parameters1);
    }

    public void start()
    {
        imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void loop()
    {
        angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu1.getGravity();

        //Check against a weird crash issue (Pitch should never equal 0 unless the robot tips over)
        //(if the program crashes, sometimes the REV Hubs report an all zero reading on the sensors and encoders.  If this happens, the following code prevents us from logging invalid values.)
        if(angles.thirdAngle != 0) {
            if (auton) {
                //In autonomous we save the live IMU angle
                angles.firstAngle = (float) LocalizationSaveState.saveLastIMUReading(angles.firstAngle, OpmodeInitialOffset, OpModeName, AutonOpModeColor);
            } else {
                //Offset by any valid last saved angle
                angles.firstAngle = (float) LocalizationSaveState.offsetYaw(angles.firstAngle);
            }

            Yaw = angles.firstAngle;
            Roll = angles.secondAngle;
            Pitch = angles.thirdAngle;
        }

        if (dataLogger != null) {
            dataLogger.addValue(Yaw, "IMU-heading");
            dataLogger.addValue(Roll, "IMU-Roll");
            dataLogger.addValue(Pitch, "IMU-Pitch");
        }

        
    }

    public void stop()
    {

    }


    public double getYaw()
    {
        return Yaw;
    }

    public double getRoll()
    {
        return Roll;
    }

    public double getPitch()
    {
        return Pitch;
    }

    public double getXAcceleration()
    {
        return XAcceleration;
    }

    public double getYAcceleration()
    {
        return YAcceleration;
    }

    public double getZAcceleration()
    {
        return ZAcceleration;
    }
}
