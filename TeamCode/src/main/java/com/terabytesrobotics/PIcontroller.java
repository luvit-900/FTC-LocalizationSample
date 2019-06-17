package com.terabytesrobotics;


/**
 * Created by carlb_000 on 11/3/2015.
 */
public class PIcontroller extends DSPTimeBased {
    double maxOutputLimit;
    double minOutputLimit;
    double Ki;
    double Kp;
    double error;
    double lastError = 0;
    double integral = 0;
    double output;
    boolean limitsMet = false;

    public void setParameters( double Ki,double Kp) {
        this.Ki = Ki;
        this.Kp = Kp;
    }

    public void setKp( double Kp) {
        this.Kp = Kp;
    }

    public void setKi( double Ki) {
        this.Ki = Ki;
    }

    public void setOutputLimit(double max, double min)
    {
        //Update the limits
        maxOutputLimit = max;
        minOutputLimit = min;

        //See if the new limits change anything
        if(output > maxOutputLimit)
        {
            //We're over the high one
            output = maxOutputLimit;

            //Adjust the integral, crudely
            if(integral > maxOutputLimit)
            {
                //If it was too high, pull it down.
                integral = maxOutputLimit;
            }
        }
        else
        {
            if(output < minOutputLimit)
            {
                //We're below the low one
                output = minOutputLimit;

                //Adjust the integral, crudely
                if(integral < maxOutputLimit)
                {
                    //If it was too low, pull it up.
                    integral = maxOutputLimit;
                }
            }
            //All is well.
        }
    }

    public double processInput(double cmd, double in) {     //reads input

        error = cmd - in; // finds error
        double speculativeIntegral;
        double speculativeOutput;
        speculativeIntegral = integral + (lastError + error) / 2 * deltaT * Ki;
        speculativeOutput = speculativeIntegral + error * Kp;
        if(speculativeOutput <= maxOutputLimit && speculativeOutput >= minOutputLimit){
            limitsMet = false;
            /****** Does the integral part of the algorithm ******/
            integral = speculativeIntegral;
            lastError = error;
            /****** Sets Output ******/
            output = speculativeOutput;
        }
        else
        {
            limitsMet = true;
            if(speculativeOutput > maxOutputLimit){
                output = maxOutputLimit;
            }
            else{
                if(speculativeOutput < minOutputLimit){
                    output = minOutputLimit;
                }
            }
        }


        return output;
    }
    public boolean getLimitsMet(){
        return limitsMet;
    }

    public double getError() {return error; }
}
