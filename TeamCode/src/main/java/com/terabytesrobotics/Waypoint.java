package com.terabytesrobotics;

/*
This class describes a waypoint for motion of the robot across the field.
 */

public class Waypoint {

    public enum TimeType
    {
        Init,               //Special time that means "This belongs to state of robot now". Time had no meaning in this case.  Current time is taken as the time of arrival here.
        FromNow,            //Time represents time from right now
        FromLastWaypoint,   //Time represents time from the last waypoint

    };

    private Coordinate coord;
    private double time;
    private TimeType timeType;
    private int id;

    //Make a Waypoint out of its components
    public Waypoint(Coordinate coord, double t, TimeType timeType, int id)
    {
        this.coord = coord;
        this.time = t;
        this.timeType = timeType;
        this.id = id;
    }

    //Make a Waypoint from another waypoint
    public Waypoint(Waypoint nutherOne)
    {
        this.coord = new Coordinate(nutherOne.coord);
        this.time = nutherOne.time;
        this.timeType = nutherOne.timeType;
        this.id = nutherOne.id;
    }

    public Coordinate getCoord()
    {
        return coord;
    }

    public double getTime()
    {
        return time;
    }

    public TimeType getTimeType()
    {
        return timeType;
    }

    public int getId()
    {
        return id;
    }
}
