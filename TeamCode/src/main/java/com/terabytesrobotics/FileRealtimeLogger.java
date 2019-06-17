package com.terabytesrobotics;

import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;


/**
 * File Realtime Logger
 * <p>
 * Logs data to a file.  For collecting real-time control and event information for later, post-loop
 * analysis.
 *
 * Usage:
 *      1) Create the log.  Good to do in the opmode init() method implementation
 *          frl.startLog("FileNameToWriteLog.txt");
 *      2) Log a row of data.  This is done once per opmode loop() iteratoin
 *          a) frl.newTimeStep();   //Do this once before logging data
 *          b) frl.addValue(fee);  frl.addValue(fie); frl.addValue(fo); //Do this as many times as is needed for all data points
 *          c) frl.endTimeStep();   //Do this once before the end of loop();
 *      3) End the log.  This is good to do in the opmode stop() method implementation.
 *          frl.endLog();
 *
 */
public class FileRealtimeLogger extends DSPTimeBased
{
    OutputStreamWriter osw;
    FileOutputStream fos;
    String headingText;
    
    public void addHeading(String heading)
    {
        if(headingText == null)
        {
            headingText = new String("Time");
        }

        headingText = headingText + "\t" + heading;
    }

    public void startLog(String filename)
    {
        File filesDir = Environment.getExternalStorageDirectory();
        try {

            fos = new FileOutputStream(new File(filesDir, filename + ".txt"));
            osw = new OutputStreamWriter(fos);

            //Put up the headings, if any
            if(headingText != null)
            {
                osw.write(headingText);
            }
            else
            {
                osw.write("Time\tUse_addHeading()_to_PutHeaddingsHere");
            }
            osw.write("\r\n");
        }

        catch (Exception e)
        {
            e.printStackTrace();
        }

    }

    private void newTimeStep(Double TIME){
        try {
            String printTime;
            printTime = Double.toString(TIME);
            osw.write(printTime);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void newTimeStep() {
        newTimeStep(nowSeconds);
    }

    public void addValue(double value){
        try {
            String printValue;
            printValue =  Double.toString(value);
            osw.write("\t" + printValue);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void addValue(int value){
        try {
            String printValue;
            printValue =  Integer.toString(value);
            osw.write("\t" + printValue);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void addValue(String value){
        try {
            String printValue;
            printValue =  value;
            osw.write("\t" + printValue);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void addValue(boolean value){
        try {
            String printValue;
            printValue =  Boolean.toString(value);
            osw.write("\t" + printValue);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    
    public void printMessage(String msg)
    {
        try {
            osw.write("\r\n");
            osw.write(msg);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void endTimeStep(){
        try {
           osw.write("\r\n");
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void endLog(){
        try {
            osw.close();
            fos.close();
            headingText = "";
        }
        catch (Exception e){
            e.printStackTrace();
        }
    }
 }
