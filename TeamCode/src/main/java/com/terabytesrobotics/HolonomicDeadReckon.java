package com.terabytesrobotics;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by A.T. Alexander on 01/31/19.
 */
 
 
/*
The description below shows the robot viewed from top.  Motors
should be connected and configured such that the behavior shown
below is produced on the machine in a way that's consistent with
the inputs.

Yf is this axis
^
|
|
| Robot Front This Side
| (This is the Yr axis)
|         ^
|         |
|     M0  |  M1
|      \  |  /
|       \ | /
|        \|/
|         +----------->  This is the Xr axis
|        / \
|       /   \
|      /     \
|     M3     M2
|
|
+----------------------------------------------------> Xf is this axis

The coordinate maintained by this class places the center of the holonomic drive
at its (x,y) coordinate on the Field (Xf, Yf).

The coordinate's thetaRad is the angle between the robot frame's Xr axis and the
field's Xf axis.

Motor directions can be understood from the following:
1) If all motors (M0..M3) have increasing counts, the thetaRad will be
decreasing; the robot will be rotating clockwise on the field when viewed
from above.

2) In a non-rotating case where all of the motor movements are the same,
if M0 & M3 are positive and M1 & M2 are negative the robot will drive
towards "Front", in the direction of its Yr axis.

3) In a non-rotating case where all of the motor movements are the same,
if M0 & M1 are positive and M2 & M3 are negative the robot will drive
towards its "Right", in the direction of its Xr axis.



Some test cases:
1) Mag > 0, thetaRAD = 90 deg, r=0 ==> Robot drives straight towards "Front".
   M1 & M4 are positive values and M2 & M3 are negative values.
2) Mag > 0, thetaRAD = 0 deg, r=0 ==> Robot drives straight towards its "Right".
   M1 & M2 are positive values and M3 & M4 are negative values.
3) Mag > 0, thetaRAD = dont-care deg, r > 0 ==> Robot rotates in place, clockwise
   when viewed from the top.

*/

public class HolonomicDeadReckon {

    private Coordinate curPosition;
    private int lastM0;
    private int lastM1;
    private int lastM2;
    private int lastM3;

    private double wheelDisplacePerEncoderCount;    //Displacement that robot will move for each encoder count
    private double holonomicCircumference;  //Circumference of the circle formed by the diagonal wheel-to-wheel distance across the holonomic drive.
    private double twoSqrtTwo;              //2.0*Sqrt(2), so that we don't have to recalculate it all the time.

    //Member data to allow external override of theta from something like an IMU.
    private double thetaOverride;
    private boolean thetaOverrideEnable;

    public HolonomicDeadReckon(double whlDispPerCount, double holoCircumference)
    {
        //Provide some default postion.
        curPosition = new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0);

        //Zero all the encoder values
        lastM0 = 0;
        lastM1 = 0;
        lastM2 = 0;
        lastM3 = 0;

        //Parameters that we'll need
        wheelDisplacePerEncoderCount = whlDispPerCount;
        holonomicCircumference = holoCircumference;

        //A constant we need a lot
        twoSqrtTwo = 2.0 * Math.sqrt(2.0);
    }

    /*
    Updates the position from new motor encoder values.
     */
    public void UpdateMotorEncoders(int m0, int m1, int m2, int m3)
    {
        //Compute change in encoder positions
        int delt_m0 = m0 - lastM0;
        int delt_m1 = m1 - lastM1;
        int delt_m2 = m2 - lastM2;
        int delt_m3 = m3 - lastM3;

        //Compute displacements for each wheel
        double displ_m0 = ((double) delt_m0) * wheelDisplacePerEncoderCount;
        double displ_m1 = ((double) delt_m1) * wheelDisplacePerEncoderCount;
        double displ_m2 = ((double) delt_m2) * wheelDisplacePerEncoderCount;
        double displ_m3 = ((double) delt_m3) * wheelDisplacePerEncoderCount;

        //Compute the average displacement in order to untangle rotation
        //from displacment
        double displ_average = (displ_m0 + displ_m1 + displ_m2 + displ_m3) / 4.0;

        //Compute the component of the wheel displaements that yield robot displacement
        double dev_m0 = displ_m0 - displ_average;
        double dev_m1 = displ_m1 - displ_average;
        double dev_m2 = displ_m2 - displ_average;
        double dev_m3 = displ_m3 - displ_average;

        //Compute the displacemnt of the holonomic drive, in robot reference frame
        double delt_Xr = (dev_m0 + dev_m1 - dev_m2 - dev_m3) / twoSqrtTwo; //1.414213;// 2.0 ;//twoSqrtTwo;
        double delt_Yr = (dev_m0 - dev_m1 - dev_m2 + dev_m3) / twoSqrtTwo; //1.414213; //2.0; //twoSqrtTwo;

        //Find the new angle of the robot
        double newRobotTheta;
        if(thetaOverrideEnable)
        {
            //Just take theta from our external source
            newRobotTheta = thetaOverride;
        }
        else
        {
            //Compute the angular rotation change from the wheels in radians
            double displ_rotation = (displ_average / holonomicCircumference) * 2.0 * Math.PI;
            //Integrate angle change computed from the wheels to get a new angle
            newRobotTheta = curPosition.getThetaRAD() - displ_rotation;
        }

        //Move this holonomic displacement from robot to field frame of reference
        double robotTheta = newRobotTheta;  //Just make the accessor call once
        double sinTheta = Math.sin(robotTheta);         //We need the sin(theta) twice
        double cosTheta = Math.cos(robotTheta);         //We need the cos(theta) twice
        double delt_Xf = delt_Xr * cosTheta - delt_Yr * sinTheta;
        double delt_Yf = delt_Yr * cosTheta + delt_Xr * sinTheta;

        //Update the position
        curPosition = new Coordinate(curPosition.getType(),
                curPosition.getX() + delt_Xf,
                curPosition.getY() + delt_Yf,
                newRobotTheta);

        //Save the current encoder values as the "last" ones
        lastM0 = m0;
        lastM1 = m1;
        lastM2 = m2;
        lastM3 = m3;
    }

    /*
    Sets the current position of the motor encoder values.  This does not
    update the position of the holonomic machine.
     */
    public void SetMotorEncoders(int m0, int m1, int m2, int m3)
    {
        lastM0 = m0;
        lastM1 = m1;
        lastM2 = m2;
        lastM3 = m3;
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
        //Make a new, replacement with the updaed x & y
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
