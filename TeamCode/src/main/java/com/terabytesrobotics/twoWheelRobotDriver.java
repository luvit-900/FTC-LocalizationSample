package com.terabytesrobotics;

import java.util.concurrent.LinkedBlockingQueue;

public class twoWheelRobotDriver {

    //State machine state
    private enum States
    {
        Idle,
        Driving,
    };
    private twoWheelRobotDriver.States state = States.Idle;

    private double maxLinearSpeed;
    private double maxLinearAcceleration;
    private double maxAngularSpeed;
    private double maxAngularAcceleration;

    //The waypoint queue
    LinkedBlockingQueue<Waypoint> waypointQueue;

    //Waypoints we're involved in.
    private Waypoint targetWaypoint;            //This is where we're driving to.  instantaneousCoordinate will move towards it
    private Coordinate instantaneousCoordinate; //This is the location we're asking to vehicle to be at right now
    private Coordinate startOfDriveCoordinate;  //This is where any given drive starts from.

    //State machine & movement management
    private boolean enabled;
    private double movePlannedTimeDelta;    //Time span we're expecting the current move to take
    private double movePlannedStartTime;    //Time that this move started.
    private double movePlannedEndTime;      //Time we're expecting the current move to end
    private double xVel;    //Dist / second
    private double yVel;    //Dist / second
    private double rotVel;  //rad / second

    //Waypoint trigger: detects when a waypoint is reached/completed
    private int waypointIDForTrigger;
    private boolean waypointTriggered;

    //Logging
    SBSDataLogger logger;

    //Constructor with logging setup
    public twoWheelRobotDriver(SBSDataLogger logger)
    {
        //Save the logger reference
        this.logger = logger;

        //Setup the queue to have a maximum capacity of 640 waypoints.  Who would ever need more
        //than that?
        waypointQueue = new LinkedBlockingQueue<Waypoint>(640);
    }
    //Basic constructor
    public twoWheelRobotDriver()
    {
        //Instantiate without the logger
        this(null);
    }

    public void init()
    {
        //Clear all of the waypoints from the queue
        flushWaypoints();
    }

    public void start()
    {
        //Empty the queue?
    }

    //Loop goes here

    public void stop()
    {

    }

    //
    // Parameters
    //
    //Max linear speed in dist/second
    public void setMaxLinearSpeed(double maxLinearSpeed)
    {
        this.maxLinearSpeed = maxLinearSpeed;
    }
    //Max linear acceleration in dist / (second^2)
    public void setMaxLinearAcceleration(double maxLinearAcceleration)
    {
        this.maxLinearAcceleration = maxLinearAcceleration;
    }
    //Max angular speed in rad/second
    public void setMaxAngularSpeed(double maxAngularSpeed)
    {
        this.maxAngularSpeed = maxAngularSpeed;
    }
    //max angular acceleration in rad/(second^2)
    public void setMaxAngularAcceleration(double maxAngularAcceleration)
    {
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    //
    // Enable/Disable
    //
    //Returns true if RobotDriver is enabled.
    public boolean getEnable()
    {
        return this.enabled;
    }
    //Enable and disable the RobotDriver
    public void setEnable(boolean enable)
    {
        this.enabled = enable;
    }

    //
    // Waypoint addition and status tracking
    //
    //Adds a waypoint to the queue.  Driving will begin at the next call to loop()
    public void addWaypoint(Waypoint newWaypoint)
    {
        waypointQueue.add(newWaypoint);
    }
    public void flushWaypoints()
    {
        waypointQueue.clear();
    }
    //Gets the id of the current waypoint that is being driven towards.  If there
    //is none, -1 is returned.
    public int getCurrentWaypointId()
    {
        if(targetWaypoint == null)
        {
            return -1;
        }
        else
        {
            return targetWaypoint.getId();
        }
    }
    //Returns true if waypoint has been reached.  That is, if the position error
    //is small enough.
    public boolean isCurrentWaypointReached()
    {
        return state == States.Idle;   //This might be a quick indication if there's another waypoint coming
    }
    //
    // Reliable detection of an arbitrary waypoint being reached.  Detection is via
    // its ID number
    //
    // Sets a trigger on a waypoint finishing
    public void setWaypointCompleteTrigger(int idForTrigger)
    {
        this.waypointIDForTrigger = idForTrigger;
        this.waypointTriggered = false;
    }
    // Tests waypoint finsishing trigger
    public boolean isWaypointCompleteTriggered(boolean clearIt)
    {
        //See if trigger was hit
        if(this.waypointTriggered)
        {
            if(clearIt)
            {
                //Clear the trigger
                this.waypointTriggered = false;
                this.waypointIDForTrigger = -1;
            }

            //Return indication that it triggered
            return true;
        }
        else
        {
            //Not triggered
            return false;
        }
    }


    //
    // Interface to the thing that's trying to control position
    //
    //Retrieves the active target point -- this is the cooridinate that position control
    //should be trying to achieve..  If there is not target, this will return a coordinate
    //of (0,0,0).
    public Coordinate getTargetPoint()
    {
        if(instantaneousCoordinate == null)
        {
            return new Coordinate(Coordinate.CoordinateType.TerabyteF, 0, 0, 0);
        }
        else
        {
            return new Coordinate(instantaneousCoordinate);
        }
    }
}
