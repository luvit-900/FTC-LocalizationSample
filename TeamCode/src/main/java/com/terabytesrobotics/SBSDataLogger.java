package com.terabytesrobotics;

import java.text.SimpleDateFormat;
import java.util.Date;

//import org.firstinspires.ftc.teamcode.opmodes.Reference.Gyro;

/**
 * Created by darwinleuba on 12/30/15.
 */
public class SBSDataLogger {

    // Use the logger we already have. No use to reinvent the wheel : )

    FileRealtimeLogger logger;
    String filename;
    boolean firstValAdded = true;
    boolean headingCollected = false;
    String referenceHeading = "";
    String currentHeading = "";

    public SBSDataLogger(String typeTag) {

        logger = new FileRealtimeLogger();

        String formatString = new String("'LOG-4149-"+typeTag+"-'yyyy-MM-dd-HHmmss");

        //Give our log a name, even though we're not starting it yet.
        //Form a timestamped filename for the log
        Date rightNow = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat(formatString);
        filename = sdf.format(rightNow);
    }

    public  void init()
    {
        //Note that we haven't collected a heading, and hence haven't started the log.
        headingCollected = false;
        
        //Empty the headings.
        referenceHeading = "";
        currentHeading = "";
    }

    public  void start(){
        // Start the time off at a low value
        DSPTimeBased.restart();
    }
    
    public void addValue(int value, String heading)
    {
        //If the heading has been collected we'll output data, otherwise we're just collecting headings
        if(headingCollected)
        {
            //If this is the first value in the loop being written to the file,
            // display the timestamp first
            if(firstValAdded)
            {
                // Display data (Five is and example)
                logger.newTimeStep();
                //Yes, we've been called at least once this loop
                firstValAdded = false;
            }
            logger.addValue(value);
            
            //Save the heading that came in so that we can detect changes in order
            //currentHeading = currentHeading + "\t" + heading;
        }
        else
        {
            //Accumulate the heading we're setting up.
            referenceHeading = referenceHeading + "\t" + heading;
            
            //Send the heading to the logger
            logger.addHeading(heading);
        }
    }
    
    public void addValue(double value, String heading)
    {
        //If the heading has been collected we'll output data, otherwise we're just collecting headings
        if(headingCollected)
        {
            if(firstValAdded)
            {
                // Display data (Five is and example)
                logger.newTimeStep();
                //Yes, we've been called at least once this loop
                firstValAdded = false;
            }
            logger.addValue(value);
            
            //Save the heading that came in so that we can detect changes in order
            //currentHeading = currentHeading + "\t" + heading;
        }
        else
        {
            //Accumulate the heading we're setting up.
            referenceHeading = referenceHeading + "\t" + heading;
            
            //Send the heading to the logger
            logger.addHeading(heading);
        }
    }
    
    public void addValue(String value, String heading)
    {
        //If the heading has been collected we'll output data, otherwise we're just collecting headings
        if(headingCollected)
        {
            if(firstValAdded)
            {
                // Display data (Five is and example)
                logger.newTimeStep();
                //Yes, we've been called at least once this loop
                firstValAdded = false;
            }
            logger.addValue(value);

            //Save the heading that came in so that we can detect changes in order
            //currentHeading = currentHeading + "\t" + heading;
        }
        else
        {
            //Accumulate the heading we're setting up.
            referenceHeading = referenceHeading + "\t" + heading;
            
            //Send the heading to the logger
            logger.addHeading(heading);
        }
    }

    public void addValue(boolean value, String heading)
    {
        //If the heading has been collected we'll output data, otherwise we're just collecting headings
        if(headingCollected)
        {
            if(firstValAdded)
            {
                // Display data (Five is and example)
                logger.newTimeStep();
                //Yes, we've been called at least once this loop
                firstValAdded = false;
            }
            logger.addValue(value);

            //Save the heading that came in so that we can detect changes in order
            //currentHeading = currentHeading + "\t" + heading;
        }
        else
        {
            //Accumulate the heading we're setting up.
            referenceHeading = referenceHeading + "\t" + heading;

            //Send the heading to the logger
            logger.addHeading(heading);
        }
    }

    //Call the loop method last of all of the opmode's loop() work.
    public  void  loop()
    {
        //If the heading was collected and output to the file, end the time step.
        if(headingCollected)
        {
            //See if the heading is the same this iteration as it was last time.
            /*
            if(!currentHeading.equals(referenceHeading))
            {
                //It's not.  Put a message in the log about that:
                logger.printMessage("Log change ^^:"+currentHeading);
            }
            */

            //End this timestep.
            logger.endTimeStep();
        }
        else
        {
            //Actually start the now.  This will cause the accumulated header to be written out.
            logger.startLog(filename);
            
            //Mark that we have collected the heading and written it
            headingCollected = true;
        }
        
        // always reset the addvalue thingy for the next loop to make sure the timestep gets reset on first value.
        firstValAdded = true;

        //Always clear the current heading that we use for finding order changes
        currentHeading = "";
    }

    public void stop(){
        // End the log and close the file
        logger.endLog();
    }
}
