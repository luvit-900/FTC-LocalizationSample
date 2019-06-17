package com.terabytesrobotics;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by darwinleuba on 11/10/16.
 */
 
 
/*
The description below shows the robot viewed from top.  Motors
should be connected and configured such that the behavior shown
below is produced on the machine in a way that's consistent with
the inputs.

 Robot Front This Side
 (thetaRAD = +90 deg)
         ^
     M1  |  M2
      \  |  /
       \ | /
        \|/
         +----> thetaRAD = 0 is this way--->>>
        / \
       /   \
      /     \
     M4     M3

Positive 'r' rotates the machine in _negative_ thetaRAD direction.
Positive 'r' rotates the machine clockwise, viewed from the top.

Some test cases:
1) Mag > 0, thetaRAD = 90 deg, r=0 ==> Robot drives straight towards "Front".
   M1 & M4 are positive values and M2 & M3 are negative values.
2) Mag > 0, thetaRAD = 0 deg, r=0 ==> Robot drives straight towards its "Right".
   M1 & M2 are positive values and M3 & M4 are negative values.
3) Mag > 0, thetaRAD = dont-care deg, r > 0 ==> Robot rotates in place, clockwise
   when viewed from the top.

*/

public class HolonomicMath {

    public HolonomicMath() {
    }

    /**
     *@param Mag mag between 0 and 1
     *@param thetaRAD theta in Radians
     *@param r rotation vector between -1 and 1
     */
    public static double[] controlsToSpeed(double Mag, double thetaRAD, double r) {

        final double rotConst = .75;

        double thetaDEG = Math.toDegrees(thetaRAD);

        //vectors are a value between 1 and 3
        double Theta1 = (thetaDEG - 45);//Front Left
        double Theta2 = (thetaDEG + 45);//Front Right
        double Theta3 = (thetaDEG + 135);//Back Right
        double Theta4 = (thetaDEG - 135);//Back Left

        Theta1 = Math.toRadians(Theta1);
        Theta2 = Math.toRadians(Theta2);
        Theta3 = Math.toRadians(Theta3);
        Theta4 = Math.toRadians(Theta4);

        double vector1 = Mag * Math.cos(Theta1) + (r*rotConst);
        double vector2 = Mag * Math.cos(Theta2) + (r*rotConst);
        double vector3 = Mag * Math.cos(Theta3) + (r*rotConst);
        double vector4 = Mag * Math.cos(Theta4) + (r*rotConst);

        double mag1 = Range.clip(vector1, -1, 1);
        double mag2 = Range.clip(vector2, -1, 1);
        double mag3 = Range.clip(vector3, -1, 1);
        double mag4 = Range.clip(vector4, -1, 1);

        double[] speeds = {mag1, mag2, mag3, mag4};

        return speeds;
    }
}