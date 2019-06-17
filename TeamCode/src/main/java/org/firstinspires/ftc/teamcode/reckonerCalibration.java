package org.firstinspires.ftc.teamcode;
import android.content.Context;
import android.os.Environment;
import android.util.Xml;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.terabytesrobotics.Coordinate;
import com.terabytesrobotics.DSPTimeBased;
import com.terabytesrobotics.FileRealtimeLogger;
import com.terabytesrobotics.LocalizationSaveState;
import com.terabytesrobotics.RobotDriver;
import com.terabytesrobotics.RobotPositionController;
import com.terabytesrobotics.SBSDataLogger;
import com.terabytesrobotics.SBSDriveMotion;
import com.terabytesrobotics.SBSIMU;
import com.terabytesrobotics.xmlReader;

import org.xmlpull.v1.XmlSerializer;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;

//Use this class to calibrate the linear component of the dead reckoner.
//In this OpMode, the left joystick will move the robot forward at a decreased speed while the position controller holds rotation.
//There are no other controls in this OpMode

/**
 * To calibrate the dead reckoner, make two parallel white tape lines on the field that are a specified distance (in inches) apart (from leading edge to leading edge).
 * Also, mount a color sensor on the bottom of the robot (this OpMode uses the REV Color/Distance sensor)
 * Then, line up the robot so that the Y axis of the robot is perpendicular to the tape lines. (line up the robot so that you drive straight over both of them)
 * Start the program and wait until telemetry says to drive.
 * Once telemetry says to drive, drive over both tape lines.
 * Check telemetry to see if both lines were found.
 * After you have driven over both lines, stop the OpMode and the program will determine the calibration value
 * If both lines are not detected, the calibration will not be written to the file.
 *
 * If the program see a line where there isn't one, try increasing the threshold.  (also make sure that the field is clean.)
 *
 * (the index number of the reading for each line is written in the cal file.  These numbers can be used to check the log.  [index starts at 0, hence line 1 is index 0])
 */

@TeleOp(name="reckonerCalibration", group="Opmode")
public class reckonerCalibration extends OpMode {
    SBSDataLogger DataLogger;
    SBSDriveMotion DriveMotion;
    SBSIMU IMU;
    RobotPositionController posCtrlr;

    //counts how many loop cycles have been completed
    private int counter = 0;

    //The distance from leading edge to leading edge of the two tape lines
    private final double lineDistacne = 36.0;

    //these values are set by the OpMode
    private double rekY_atFirstLine = 0.0;
    private double rekY_atSecondLine = 0.0;
    private static double calValue;
    private int firstLineIndex;
    private int secondLineIndex;

    //DRIVE CALIBRATION COLOR SENSOR
    ColorSensor driveCalLightSensor;

    //lag 10 for the smoothing functions
    int lag = 10;
    //3.5 standard deviations for signal
    double threshold = 25.0;

    //Initialise variables
    double avgFilter;

    boolean firstLineFound;
    boolean secondLineFound;

    int lastSignal = 0;

    @Override
    public void init()
    {
        DataLogger = new SBSDataLogger("LC");  //  LC = Linear Calibration
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

        DriveMotion.init();
        IMU.init();

        posCtrlr.setKpGainRot(0.5);
        posCtrlr.setKiGainRot(0.0);

        //set initial coordinate to 0
        try {
            DriveMotion.getReckoner().SetHolonomicPosition(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0));
        } catch (Exception e) {

        }

        driveCalLightSensor = hardwareMap.get(ColorSensor.class, "driveCalLightSensor");
        driveCalLightSensor.enableLed(true);
    }
    @Override
    public void start()
    {
        DSPTimeBased.restart();
        DataLogger.start();
        DriveMotion.start();
        IMU.start();
    }
    @Override
    public void loop()
    {
        DSPTimeBased.updateTimestamp();

        IMU.loop();

        //retrieve color sensor data
        int alpha = driveCalLightSensor.alpha();
        int argb = driveCalLightSensor.argb();
        int red = driveCalLightSensor.red();
        int green = driveCalLightSensor.green();
        int blue = driveCalLightSensor.blue();

        //log data from the color sensor
        DataLogger.addValue(alpha, "driveCalSensor-alpha");
        DataLogger.addValue(argb, "driveCalSensor-argb");
        DataLogger.addValue(red, "driveCalSensor-red");
        DataLogger.addValue(green, "driveCalSensor-green");
        DataLogger.addValue(blue, "driveCalSensor-blue");

        //get gamepad values
        double driveY = -gamepad1.left_stick_y * 0.3;

        //Use posCtrlr to hold robot rotation
        //Pass in the data about where we want the robot to be.
        Coordinate targPoint = new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0);
        posCtrlr.setRobotTargetPosition(targPoint);
        //Pass in the data about where the robot is.
        Coordinate drPoint = DriveMotion.getReckoner().GetHolonomicPosition();
        posCtrlr.setRobotFeedbackPosition(drPoint);
        //Compute the controller outputs
        posCtrlr.compute();
        //posCtrlr generates a positive speed for increasing angle.  Drive code moves clockwise (decreases in angle) when given a positive speed (was built this way for gamepad controls).
        DriveMotion.setR(-posCtrlr.getRotVel());

        //send gamepad values to drive code
        DriveMotion.cartesianInput(0.0, driveY);

        //Execute drive code
        DriveMotion.loop();

        //Look for the lines
        if(counter < lag)
        {
            avgFilter += alpha;
            telemetry.addData("Don't drive yet!","");
        }
        else if(counter == lag)
        {
            avgFilter = avgFilter / lag;
            telemetry.addData("Drive!","");
        }
        else
        {
            telemetry.addData("Drive!","");
        }

        if(counter >= lag)
        {
            if (Math.abs(alpha - avgFilter) > threshold)
            {
                if (alpha > avgFilter)
                {
                    if(lastSignal != 1) // make sure that we move off of the first line before looking for the second.
                    {
                        if (!firstLineFound)
                        {
                            rekY_atFirstLine = DriveMotion.getReckoner().GetHolonomicPosition().getY();
                            firstLineIndex = counter;  //for debugging
                            firstLineFound = true;
                        }
                        else if (!secondLineFound)
                        {
                            rekY_atSecondLine = DriveMotion.getReckoner().GetHolonomicPosition().getY();
                            secondLineIndex = counter;  //for debugging
                            secondLineFound = true;
                        }
                    }
                    lastSignal = 1;
                }
                else
                {
                    lastSignal = -1;
                }
            }
            else
            {
                lastSignal = 0;
            }
        }

        //put the index of each data point in the log for debugging.
        DataLogger.addValue(counter, "dataIndex");

        telemetry.addData("DSP-Time: ", DSPTimeBased.getNowSeconds());
        telemetry.addData("", "");
        telemetry.addData("FirstLineFound?", firstLineFound);
        telemetry.addData("SecondLineFound?", secondLineFound);
        if(firstLineFound && secondLineFound)
        {
            telemetry.addData("", "");
            telemetry.addData("Calibration complete!","");
        }
        telemetry.update();

        counter++;

        //Datalogger's loop() is last of all loop() work
        DataLogger.loop();
    }
    @Override
    public void stop()
    {
        if(firstLineFound && secondLineFound)
        {
            calibrateReckoner();
        }
        IMU.stop();
        DriveMotion.stop();
        DataLogger.stop();
    }

    private void calibrateReckoner()
    {
        calValue = lineDistacne / (rekY_atSecondLine - rekY_atFirstLine);

        //Write Cal Value to a file
        File filesDir = Environment.getExternalStorageDirectory();
        try {
            FileOutputStream fos = new  FileOutputStream(new File(filesDir, RobotDefinitions.rekCalFilename));
            XmlSerializer xmlSerializer = Xml.newSerializer();
            StringWriter writer = new StringWriter();
            xmlSerializer.setOutput(writer);
            xmlSerializer.startDocument("UTF-8", true);
            xmlSerializer.startTag(null, "rekCal");
            xmlSerializer.startTag(null, "calVal");
            xmlSerializer.text(Double.toString(calValue));
            xmlSerializer.endTag(null, "calVal");
            xmlSerializer.startTag(null,"firstLineIndex");
            xmlSerializer.text(Integer.toString(firstLineIndex));
            xmlSerializer.endTag(null, "firstLineIndex");
            xmlSerializer.startTag(null, "secondLineIndex");
            xmlSerializer.text(Integer.toString(secondLineIndex));
            xmlSerializer.endTag(null, "secondLineIndex");
            xmlSerializer.endTag(null, "rekCal");
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