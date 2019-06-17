package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.comp.Todo;
import com.terabytesrobotics.Coordinate;
import com.terabytesrobotics.xmlReader;

public class RobotDefinitions {

    //  ROBOT SEEN FROM ABOVE
    //
    //      X           X
    //     X             X
    //    X  W1       W2  X
    //            X
    //           XXX
    //            X
    //    X  W4       W3  X
    //      X           X
    //        X       X
    //

    public final static String IMU1 = "imu1";

    // Drive motors
    final static public String wheelOneMotor = "motor1";
    final static public String wheelTwoMotor = "motor2";
    final static public String wheelThreeMotor = "motor3";
    final static public String wheelFourMotor = "motor4";


    public enum AutonOpModeColor
    {
        RED,
        BLUE,
        NONE
    }

    /***** TODO: LOOK AT THE CODE BELOW AND CHANGE OR REMOVE UNECESSARY CODE ASAP ****************/


    public enum AutonGoldLocation
    {
        Left,
        Center,
        Right,
        UNKNOWN
    }



    public final static double RedCrater_AutonOffsetAngle = -45.0;
    public final static double BlueCrater_AutonOffsetAngle = 135.0;
    public final static double RedCorner_AutonOffsetAngle = 45.0;
    public final static double BlueCorner_AutonOffsetAngle = 225.0;

    public final static String rekCalFilename = "rekCal.xml";
    public final static String rotCalFilename = "rotCal.xml";

    //Drive system parameters
    //-Encoder count to field displacment conversion
    private final static double driveWheelDiameter = 4.0;    //Inches
    private final static double driveWheelCircumfrence = driveWheelDiameter * Math.PI;
    private final static double driveWheelMotorGearbox = 60.0; //60 encoder revolutions to "motor" shaft revolutions
    private final static double driveWheelGearOnMotor = 80.0;    //80 teeth on gear on the "motor" shaft.
    private final static double driveWheelGearOnWheel = 40.0;    //40 teeth on gear attached to wheel.
    private final static double driveWheelExtGearbox = driveWheelGearOnWheel / driveWheelGearOnMotor;
    private final static double driveWheelTotalRatio = driveWheelMotorGearbox * driveWheelExtGearbox;
    private final static double driveWheelEncoderPulsePerRev = 7.0;  //Pulse is a rising->high->falling->low cycle.
    private final static double driveWheelEncoderCountPerRev = driveWheelEncoderPulsePerRev * 4.0; //Quadrature makes 7 pulse / rev into 28 count / rev
    private final static double driveWheelCountsPerRev = driveWheelEncoderCountPerRev * driveWheelTotalRatio;
    private static double driveWheelCalibration = 1.0;
    public static double driveWheelDistancePerCount = (driveWheelCircumfrence / driveWheelCountsPerRev) * driveWheelCalibration;
    //-Robot dimensions
    private final static double robotDiagonalDiameterOfWheels = 17.0868;    //Inches
    private static double rotCalibration = 1.0;
    public static double robotWheelCircleCircumfrence = Math.PI * robotDiagonalDiameterOfWheels;

    public static void refreshCalValues()
    {
        refreshLinearCalValue();
        refreshRotationCalValue();
    }

    //FOR USE IN CALIBRATION OPMODES ONLY
    public static void refreshLinearCalValue()
    {
        driveWheelCalibration = xmlReader.readXML(rekCalFilename, "calVal");
        driveWheelDistancePerCount = (driveWheelCircumfrence / driveWheelCountsPerRev) * driveWheelCalibration;
    }

    //FOR USE IN CALIBRATION OPMODES ONLY
    public static void refreshRotationCalValue()
    {
        rotCalibration = 1.0 / xmlReader.readXML(rotCalFilename, "calVal");
        robotWheelCircleCircumfrence = Math.PI * robotDiagonalDiameterOfWheels * rotCalibration;
    }
}
