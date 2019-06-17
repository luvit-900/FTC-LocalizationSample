package com.terabytesrobotics;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Andrew Alexander on 11/7/2015.
 *
 * This class is meant as a base class for anything that needs to compute
 * relative to time passing, in a digital-signal-processing (DSP) way.
 *
 * Invoke the class (static) method updateTimestamp() once at the beginning
 * of the OpMode's loop() method to update time for all of the derived classes
 * that will use it afterward.
 */
public class DSPTimeBased {
    private static boolean calledOnce = false;  //Set once we're updated once
    protected static double nowSeconds;         //Timestamp from when updateTimestamp() was invoked
    protected static double lastSeconds;        //Timestamp from the last time updateTimestamp was
                                                // invoked
    protected static double deltaT;             //Pre-computed (now-last) value
    protected static double divByDeltaT;        //Pre-computed 1/(now - last) value.

    //An object that keeps track of time for us and all of our derived classes
    private static ElapsedTime elapsedTime = new ElapsedTime();

    public static void restart()
    {
        //Restart the time to allow us to loop more than once in a row
        //without having screwed up time deltas
        calledOnce = false;
    }

    public static void updateTimestamp() {

        //Save our old "now" timestamp as our old timestamp.
        DSPTimeBased.lastSeconds = DSPTimeBased.nowSeconds;

        //Get our new "now" timestamp.
        DSPTimeBased.nowSeconds = elapsedTime.time();

        //If we've never been updated before, the current time from elapsedTime is going to be from
        //when we were instantiated (like when OpMode was selected or maybe init() called) until
        //now, which could be minutes!  Make up our first delta-T to be something reasonable
        //and typical
        if(!DSPTimeBased.calledOnce)
        {
            //Make up a delta-T of 5ms
            DSPTimeBased.deltaT = 0.005;
        }
        else
        {
            //Compute delta-T from our usual source because some need this
            DSPTimeBased.deltaT = DSPTimeBased.nowSeconds - DSPTimeBased.lastSeconds;
        }

        //Compute the inverse of delta-T, because others need this
        DSPTimeBased.divByDeltaT = 1.0/ DSPTimeBased.deltaT;

        //Yes, we've been called at least once
        DSPTimeBased.calledOnce = true;
    }

    public static double getDeltaT() {
        return deltaT;
    }
    public static double getNowSeconds() {
        return DSPTimeBased.nowSeconds;
    }
}
