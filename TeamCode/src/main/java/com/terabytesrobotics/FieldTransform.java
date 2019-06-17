package com.terabytesrobotics;

public class FieldTransform {

    /*
    Takes point on the field "inField" and makes it relative to the position and orientation of
    the robot on that same field "inRobot".
     */
    public static Coordinate toRobot(Coordinate inField, Coordinate inRobot)
    {
        //Bad error checking here.  We assume that the inRobot cooridinate is in the same
        //x,y reference as the inField one and that the inRobot angle represents angle relative
        //to the field.
        //First compute the position of the field point relative to the robot
        //position on the field.  These are in (delt_Xf, delt_Yf)
        double delt_Xf = inField.getX() - inRobot.getX();
        double delt_Yf = inField.getY() - inRobot.getY();

        //Precompute sin/cos of theta taken from the robot angle relative to the field
        double thetaRads = inRobot.getThetaRAD();
        double sinTheta = Math.sin(thetaRads);
        double cosTheta = Math.cos(thetaRads);

        //Rotate the point on the field
        double delt_Xr = delt_Xf * cosTheta + delt_Yf * sinTheta;
        double delt_Yr = delt_Yf * cosTheta - delt_Xf * sinTheta;

        //The returned coordinate is just a point, relative to the robot.  The angle doesn't mean anything.
        return new Coordinate(Coordinate.CoordinateType.Robot,
                delt_Xr,
                delt_Yr,
                0.0);
    }

    /*
    Takes point relative to the robot "relRobot" on the robot with position and orientation of
    the robot on that field "inRobot".  Returned point is on the field of the same reference (assumted TerabyteF)
    as the (x,y) from "inRobot"
     */
    public static Coordinate toField(Coordinate relRobot, Coordinate inRobot)
    {
        //Bad error checking here.  We assume that the inRobot cooridinate is in the same
        //x,y reference as the resulting point.  We're calling that TerabyteF.
        // The angle is ignroed from relRobot.

        //Precompute sin/cos of theta taken from the robot angle relative to the field
        double thetaRads = inRobot.getThetaRAD();
        double sinTheta = Math.sin(thetaRads);
        double cosTheta = Math.cos(thetaRads);

        //Grab the point that's relative to the robot's x&y
        double delt_Xr = relRobot.getX();
        double delt_Yr = relRobot.getY();

        //Rotate the point onto the field (This is like a Park transform)
        double delt_Xf = delt_Xr * cosTheta - delt_Yr * sinTheta;
        double delt_Yf = delt_Yr * cosTheta + delt_Xr * sinTheta;

        //The returned coordinate is just a point, relative to the robot, on the field.  The angle doesn't mean anything in the returned value
        return new Coordinate(Coordinate.CoordinateType.TerabyteF,
                delt_Xf + inRobot.getX(),
                delt_Yf + inRobot.getY(),
                0.0);
    }

    public static Coordinate toTerabyte(Coordinate in) // from Vuforia field reference coordinates to Terabytes field reference coordinates
    {
        if(in.getType() == Coordinate.CoordinateType.VuforiaF)
        {
            double outX = -in.getY();
            double outY = in.getX() + 72;
            double outTheta = in.getThetaRAD() + (3.14159265358979323846264338327950288419716939937510582097494/2);
            return new Coordinate(Coordinate.CoordinateType.TerabyteF, outX, outY, outTheta);
        }
        else if(in.getType() == Coordinate.CoordinateType.TerabyteF)
        {
            return in;
        }
        else
        {
            //throw new Exception("Non field oriented coordinate!");
            return null;
        }
    }

    public static Coordinate toVuforia(Coordinate in) // from Terabytes field reference coordinates to Vuforia field reference coordinates
    {
        if(in.getType() == Coordinate.CoordinateType.TerabyteF)
        {
            double outX = -in.getY();
            double outY = in.getX() - 72;
            double outTheta = in.getThetaRAD() - (3.14159265358979323846264338327950288419716939937510582097494/2);
            return new Coordinate(Coordinate.CoordinateType.TerabyteF, outX, outY, outTheta);
        }
        else if(in.getType() == Coordinate.CoordinateType.VuforiaF)
        {
            return in;
        }
        else
        {
            //throw new Exception("Non field oriented coordinate!");
            return null;
        }
    }
}
