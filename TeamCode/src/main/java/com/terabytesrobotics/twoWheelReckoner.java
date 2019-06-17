package com.terabytesrobotics;

public class twoWheelReckoner {

    private Coordinate curPosition;
    private int lastM0;
    private int lastM1;

    private double wheelDisplacePerEncoderCount;    //Displacement that robot will move for each encoder count
    private double holonomicCircumference;  //Circumference of the circle formed by the diagonal wheel-to-wheel distance across the holonomic drive.
    private double twoSqrtTwo;              //2.0*Sqrt(2), so that we don't have to recalculate it all the time.

    //Member data to allow external override of theta from something like an IMU.
    private double thetaOverride;
    private boolean thetaOverrideEnable;

    public twoWheelReckoner(double whlDispPerCount, double holoCircumference)
    {
        //Provide some default postion.
        curPosition = new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0);

        //Zero all the encoder values
        lastM0 = 0;
        lastM1 = 0;

        //Parameters that we'll need
        wheelDisplacePerEncoderCount = whlDispPerCount;
        holonomicCircumference = holoCircumference;

        //A constant we need a lot
        twoSqrtTwo = 2.0 * Math.sqrt(2.0);
    }

    /*
    Updates the position from new motor encoder values.
     */
    public void UpdateMotorEncoders(int m0, int m1)
    {
        //Compute change in encoder positions
        int delt_m0 = m0 - lastM0;
        int delt_m1 = m1 - lastM1;

        //Compute displacements for each wheel
        double displ_m0 = ((double) delt_m0) * wheelDisplacePerEncoderCount;
        double displ_m1 = ((double) delt_m1) * wheelDisplacePerEncoderCount;

        double deltaTheta = 0.0;

        //Find the new angle of the robot
        double newRobotTheta;
        if(thetaOverrideEnable)
        {
            //Just take theta from our external source
            newRobotTheta = thetaOverride;
            deltaTheta = newRobotTheta - curPosition.getThetaRAD();
        }
        else
        {
            double displ_average = (displ_m0 + displ_m1) / 2.0;
            //Compute the angular rotation change from the wheels in radians
            double displ_rotation = (displ_average / holonomicCircumference) * 2.0 * Math.PI;
            //Integrate angle change computed from the wheels to get a new angle
            newRobotTheta = curPosition.getThetaRAD() - displ_rotation;
            deltaTheta = newRobotTheta - curPosition.getThetaRAD();
        }

        //Move this holonomic displacement from robot to field frame of reference
        double motionArc = (displ_m0 - displ_m1) / 2.0;
        double motionRadius = motionArc / deltaTheta;

        //make delta theta always be from the x axis
        if(motionArc > 0.0)
        {
            if(deltaTheta > 0)
            {
                //do nothing
            }
            else
            {
                deltaTheta = deltaTheta + Math.PI;
            }
        }
        else
        {
            if(deltaTheta > 0.0)
            {
                deltaTheta = deltaTheta + Math.PI;
            }
            else
            {
                //do nothing
            }
        }
        double x = Math.cos(deltaTheta);
        double y = Math.sin(deltaTheta);

        double delt_Xf;
        double delt_Yf;

        if(x > 0)
        {
            delt_Xf = motionRadius * (1.0 - x);
        }
        else
        {
            delt_Xf = motionRadius * (-1.0 - x);
        }

        delt_Yf = motionRadius * y;

        //Update the position
        curPosition = new Coordinate(curPosition.getType(),
                curPosition.getX() + delt_Xf,
                curPosition.getY() + delt_Yf,
                newRobotTheta);

        //Save the current encoder values as the "last" ones
        lastM0 = m0;
        lastM1 = m1;
    }

    /*
    Sets the current position of the motor encoder values.  This does not
    update the position of the holonomic machine.
     */
    public void SetMotorEncoders(int m0, int m1)
        {
        lastM0 = m0;
        lastM1 = m1;
    }

    /*
    Set the current position of the robot, with all three degrees of freedom.
    Cooridinate must be CoordinateType.TerabytesF or CoordinateType.VuforiaF and
    is retained.
     */
    public void SetHolonomicPosition(Coordinate newPosition) throws Exception
    {
        if(newPosition.isSameType(Coordinate.CoordinateType.TerabyteF) ||
                newPosition.isSameType(Coordinate.CoordinateType.VuforiaF))
        {
            curPosition = newPosition;
        }
        else
        {
            throw new Exception("Prohibited cooridinate type");
        }
    }

    /*
    Set the current field position of the robot without affecting the saved
    rotational orientation.  Cooridinate system is not affected.
     */
    public void SetHolonomicPosition(double x, double y)
    {
        //Make a new, replacement with the updated x & y
        curPosition = new Coordinate(curPosition.getType(),
                x,
                y,
                curPosition.getThetaRAD());
    }

    /*
    Set the current rotational orientation of the robot without affecting the saved
    position on the field.  Note that theta is IN RADIANS!!!!!!!!!!!!!!
     */
    public void SetHolonomicPosition(double theta)
    {
        //Make a new, replacement with the updaed x & y
        curPosition = new Coordinate(curPosition.getType(),
                curPosition.getX(),
                curPosition.getY(),
                theta);
    }

    /*
    Set an override of angle to be used in place of the dead reckoning computed angle.  This
    setter is intended for something like using an IMU angle.

    Note that theta is IN RADIANS!!!!!!!!!!!!!!
     */
    public void SetHolonomicAngleOverride(double theta, boolean bEnable)
    {
        thetaOverride = theta;
        thetaOverrideEnable = bEnable;
    }

    /*
        Retrieves the current dead-reckoned postion of the robot.
    */
    public Coordinate GetHolonomicPosition()
    {
        return curPosition;
    }
}
