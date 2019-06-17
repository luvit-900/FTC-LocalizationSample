package com.terabytesrobotics;

import org.firstinspires.ftc.teamcode.RobotDefinitions;

/**
 * Created by Ronald on 1/19/2018.
 *
 * The field is treated so that the Y axis runs from the audience to the far cryptoboxes
 *  and the X axis runs from the audience's left to the audience's right.
 *
 */

public class LocalizationSaveState {

    private static boolean wasSaved = false;
    private static String lastOpMode;
    private static double lastUseTimeStamp;
    private static double lastIMUReading;
    private static double OpModeAngleOffset;
    private static RobotDefinitions.AutonOpModeColor AutonOpModeColor;
    private static Coordinate lastLocalizationPos;

    private static final double angleStaleTimeout = 2.0 * 60;  //2.0 minutes

    private static double FixAngleRange(double in)
    {
        double retval;

        //Make sure angle is in -180 to +360 range
        in %= 360.0;

        if(in > 180.0)
        {
            retval = in - 360.0;
        }
        else
        {
            retval = in;
        }

        return retval;
    }

    public static double saveLastIMUReading(double IMUReading, double OpModeInitialOffset, String OpModeName, RobotDefinitions.AutonOpModeColor OpModeColor)
    {
        lastIMUReading = IMUReading;
        wasSaved = true;
        lastOpMode = OpModeName;
        lastUseTimeStamp = DSPTimeBased.getNowSeconds();
        OpModeAngleOffset = OpModeInitialOffset;
        if(OpModeColor != RobotDefinitions.AutonOpModeColor.NONE)
        {
            AutonOpModeColor = OpModeColor;
        }

        return FixAngleRange(lastIMUReading + OpModeAngleOffset);

    }

    public static double offsetYaw(double Yaw)
    {

        //Make sure that we actually saved some values and that any saved values are not too old
        if (wasSaved && (Math.abs(lastUseTimeStamp - DSPTimeBased.getNowSeconds()) < angleStaleTimeout))
        {
            double rawOffsetYaw = Yaw + lastIMUReading + OpModeAngleOffset;
            double offsetYaw;

            offsetYaw = FixAngleRange(rawOffsetYaw);


            // makes time always valid after valid once.
            lastUseTimeStamp = DSPTimeBased.getNowSeconds();

            return offsetYaw;
        }
        else
        {
            AutonOpModeColor = RobotDefinitions.AutonOpModeColor.NONE;
            return Yaw;
        }
    }

    public static RobotDefinitions.AutonOpModeColor getAutonOpModeColor(){ return AutonOpModeColor; }

    public static void saveRobotFieldLocation(Coordinate location)
    {
        lastLocalizationPos = location;
        wasSaved = true;
        lastUseTimeStamp = DSPTimeBased.getNowSeconds();
    }

    public static Coordinate getLastLocalizationPos()
    {
        if (wasSaved && (Math.abs(lastUseTimeStamp - DSPTimeBased.getNowSeconds()) < angleStaleTimeout)) {
            return lastLocalizationPos;
        }
        else
        {
            //If no coordinate was saved, return this coordinate.
            return new Coordinate(Coordinate.CoordinateType.TerabyteF, 24.0, 48.0, 0.0);
        }
    }
}
