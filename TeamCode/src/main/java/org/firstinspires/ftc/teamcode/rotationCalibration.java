package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Xml;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.terabytesrobotics.Coordinate;
import com.terabytesrobotics.DSPTimeBased;
import com.terabytesrobotics.LocalizationSaveState;
import com.terabytesrobotics.RobotDriver;
import com.terabytesrobotics.RobotPositionController;
import com.terabytesrobotics.SBSDataLogger;
import com.terabytesrobotics.SBSDriveMotion;
import com.terabytesrobotics.SBSIMU;
import com.terabytesrobotics.Waypoint;
import com.terabytesrobotics.xmlReader;

import org.xmlpull.v1.XmlSerializer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.StringWriter;

//Use this class to calibrate the rotational component of the dead reckoner.
//There are no user controls in this OpMode.  The robot starts moving when you press start.

/**
 * IMPORTANT: Calibrate the IMU before using this OpMode (See SensorBNO055IMUCalibration.java)  --  Also make sure that SetHolonomicAngleOverride in SBSDriveMotion is disabled
 *            Perform the linear calibration before using this OpMode.
 * To calibrate the dead reckoner, place the robot on the foam tile field.
 * After telemetry reports that the calibration is done, stop the OpMode and the program will determine the calibration value
 * If the robot is stopped before telemetry reports that the calibration is done, nothing will be written to the file.
 */

@TeleOp(name="rotationCalibration", group="Opmode")
public class rotationCalibration extends OpMode {
    //instantiate subsystems
    SBSDataLogger DataLogger;
    SBSDriveMotion DriveMotion;
    SBSIMU IMU;
    RobotDriver arde;   //Pronounced "ar-dee", this driver's name
    RobotPositionController posCtrlr;

    private double calValue;
    //in Radians
    private double rekTheta;
    private double imuTheta;

    @Override
    public void init()
    {
        RobotDefinitions.refreshLinearCalValue();
        DataLogger = new SBSDataLogger("RC");
        arde = new RobotDriver(DataLogger); //Instantiate the robot driver
        IMU = new SBSIMU(hardwareMap.get(BNO055IMU.class, RobotDefinitions.IMU1),
                false,
                "driveTest",
                0.0,        //zeroes IMU, assumed Xr || Xf (and Yr || Yf) orientation
                LocalizationSaveState.getAutonOpModeColor(),
                DataLogger);
        DriveMotion = new SBSDriveMotion((DcMotorEx) hardwareMap.get(RobotDefinitions.wheelOneMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelTwoMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelThreeMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelFourMotor),
                DataLogger,
                IMU);

        posCtrlr = new RobotPositionController(DataLogger);

        arde.init();
        DriveMotion.init();
        IMU.init();

        posCtrlr.setKpGainX(0.08);
        posCtrlr.setKiGainX(0.0);
        posCtrlr.setKpGainY(0.08);
        posCtrlr.setKiGainY(0.0);
        posCtrlr.setKpGainRot(0.5 * 0.4);
        posCtrlr.setKiGainRot(0.0005);

        posCtrlr.setLinearControlTolerance(1.0); //inches
        posCtrlr.setRotationalControlTolerance(0.08); //radians

        //set initial coordinate to 0, 0
        try {
            DriveMotion.getReckoner().SetHolonomicPosition(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0));
        } catch (Exception e) {

        }
        telemetry.addData("Press Start to Calibrate.","");
        telemetry.update();
    }
    @Override
    public void start()
    {
        DSPTimeBased.restart();
        arde.start();
        DataLogger.start();
        DriveMotion.start();
        IMU.start();

        //initial position
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0,  0,   0), 0, Waypoint.TimeType.Init, 0));
        //first waypoint (rotate 2PI RAD [360 DEG])
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0, 0,   2*Math.PI), 7.5, Waypoint.TimeType.FromLastWaypoint, 1));

        //Turn on Arde, the waypoint driver
        arde.setEnable(true);
    }
    @Override
    public void loop()
    {
        DSPTimeBased.updateTimestamp();

        IMU.loop();

        arde.loop();
        //Pass in the data about where we want the robot to be.
        Coordinate targPoint = arde.getTargetPoint();
        posCtrlr.setRobotTargetPosition(targPoint);
        //Pass in the data about where the robot is.
        Coordinate drPoint = DriveMotion.getReckoner().GetHolonomicPosition();
        //use the IMU for rotation for feedback to the position controller so that the robot actually rotates 360
        drPoint = new Coordinate(Coordinate.CoordinateType.TerabyteF, drPoint.getX(), drPoint.getY(), Math.toRadians(IMU.getYaw()));
        posCtrlr.setRobotFeedbackPosition(drPoint);
        //Compute the controller outputs
        posCtrlr.compute();
        //posCtrlr generates a positive speed for increasing angle.  Drive code moves clockwise (decreases in angle) when given a positive speed (was built this way for gamepad controls).
        DriveMotion.setR(-posCtrlr.getRotVel());
        DriveMotion.polarInput(posCtrlr.getLinVelMag(), posCtrlr.getLinVelTheta());

        DriveMotion.loop();

        //Grab these values for the calibration
        imuTheta = Math.toRadians(IMU.getYaw());
        rekTheta = DriveMotion.getReckoner().GetHolonomicPosition().getThetaRAD();

        telemetry.addData("DSP-Time: ", DSPTimeBased.getNowSeconds());
        telemetry.addData("Yaw", IMU.getYaw());
        //ask the position controller if we are there and display the answer on Telemetry.
        telemetry.addData("Cal_Done?  ", posCtrlr.getPositionInControl());
        telemetry.update();
        //Datalogger's loop() is last of all loop() work
        DataLogger.loop();
    }
    @Override
    public void stop()
    {
        if(posCtrlr.getPositionInControl())
        {
            calibrateReckoner();
        }

        IMU.stop();
        DriveMotion.stop();
        DataLogger.stop();
    }

    private void calibrateReckoner()
    {
        //rekTheta should = 2*PI;  imuTheta should = 0.0;  Add two PI to imuTheta to make them match
        calValue = (imuTheta + (2*Math.PI)) / rekTheta;

        //Write Cal Value to a file
        File filesDir = Environment.getExternalStorageDirectory();
        try {
            FileOutputStream fos = new  FileOutputStream(new File(filesDir, RobotDefinitions.rotCalFilename));
            XmlSerializer xmlSerializer = Xml.newSerializer();
            StringWriter writer = new StringWriter();
            xmlSerializer.setOutput(writer);
            xmlSerializer.startDocument("UTF-8", true);
            xmlSerializer.startTag(null, "rotCal");
            xmlSerializer.startTag(null, "calVal");
            xmlSerializer.text(Double.toString(calValue));
            xmlSerializer.endTag(null, "calVal");
            xmlSerializer.endTag(null, "rotCal");
            xmlSerializer.endDocument();
            xmlSerializer.flush();
            String dataWrite = writer.toString();
            fos.write(dataWrite.getBytes());
            fos.close();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
}
