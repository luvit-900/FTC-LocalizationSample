package com.terabytesrobotics;

/*
RobotDriver manages waypoints and drives a position setpoint between them at the specified time or
using the defined max speeds and accelerations.
 */


import java.util.concurrent.LinkedBlockingQueue;

public class RobotDriver
{
    //State machine state
    private enum States
    {
        Idle,
        Driving,
    };
    private RobotDriver.States state = States.Idle;

    //Parameters
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
    public RobotDriver(SBSDataLogger logger)
    {
        //Save the logger reference
        this.logger = logger;

        //Setup the queue to have a maximum capacity of 640 waypoints.  Who would ever need more
        //than that?
        waypointQueue = new LinkedBlockingQueue<Waypoint>(640);
    }
    //Basic constructor
    public RobotDriver()
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

    public void loop()
    {
        //States: Disabled, Idle/holding, accelerating, driving constant, declerating.
        //Internal:

        // State machine suspends when disabled.  Enable just causes it to resume where it left off.
        if(this.enabled)
        {
            switch (state)
            {
                case Idle:
                    //See if we have a waypoint read for us
                    Waypoint nextWaypoint = waypointQueue.peek();
                    if(nextWaypoint != null)
                    {
                        //We have a new waypoint remove it from the queue
                        try
                        {
                            nextWaypoint = waypointQueue.take();
                        }
                        catch(Exception InterruptedException)
                        {
                            //TODO: Just ditch this.  Or something?
                        }

                        //See what we can do, based on some stuff
                        Waypoint.TimeType timeType = nextWaypoint.getTimeType();
                        if(timeType == Waypoint.TimeType.Init)
                        {
                            //This is an initial waypoint.  Capture the coordiante as where we are
                            //and the time now to go with it.  Make deep copies of this as our
                            //current position target and instanenous target
                            targetWaypoint = new Waypoint(nextWaypoint);
                            instantaneousCoordinate = new Coordinate(nextWaypoint.getCoord());
                            //Snag the current time as the time of this waypoint
                            //We haven't moved so the last move end time is now
                            movePlannedEndTime = DSPTimeBased.getNowSeconds();
                        }
                        else
                        {
                            //This isn't an initial position waypoint, make sure that we have
                            //a current position
                            if((instantaneousCoordinate != null) &&
                            (targetWaypoint != null))
                            {
                                //Set up our new target waypoint by making a copy
                                targetWaypoint = new Waypoint(nextWaypoint);

                                //Grab the time now as the start of this move
                                movePlannedStartTime = DSPTimeBased.getNowSeconds();

                                //Figure out what our end time will be.
                                double thisMovePlannedEndTime;
                                if(nextWaypoint.getTimeType() == Waypoint.TimeType.FromNow)
                                {
                                    //Waypoint is time difference from right now.
                                    thisMovePlannedEndTime =  nextWaypoint.getTime() + movePlannedStartTime;
                                }
                                else
                                {
                                    //Assume this was delta from last waypoint end time.  We'll find this in our
                                    //member var for the planned end time.
                                    thisMovePlannedEndTime = nextWaypoint.getTime() + movePlannedEndTime;

                                    //Make sure we're not going back in time too much
                                    if(thisMovePlannedEndTime < (nextWaypoint.getTime() + movePlannedStartTime))
                                    {
                                        //We didn't take this point until too late.  Slip it's time so that there's
                                        //at least enought time for it.
                                        thisMovePlannedEndTime =  nextWaypoint.getTime() + movePlannedStartTime;
                                    }
                                }
                                movePlannedEndTime = thisMovePlannedEndTime;

                                //Figure out how long this move will take.
                                movePlannedTimeDelta = movePlannedEndTime - movePlannedStartTime;

                                //Remember where we started from so we can figure out where we should be over time
                                startOfDriveCoordinate = new Coordinate(instantaneousCoordinate);

                                //Figure out our velocities
                                xVel = (targetWaypoint.getCoord().getX() - instantaneousCoordinate.getX()) / movePlannedTimeDelta;
                                yVel = (targetWaypoint.getCoord().getY() - instantaneousCoordinate.getY()) / movePlannedTimeDelta;
                                rotVel = (targetWaypoint.getCoord().getThetaRAD() - instantaneousCoordinate.getThetaRAD()) / movePlannedTimeDelta;

                                //Go drive the instantaneousWaypoint until enough time has gone by
                                state = States.Driving;
                            }
                            //One of instantaneous or target doesn't exist.  Skip this waypoint and
                            //see if another one shows up.
                        }
                    }
                    break;

                case Driving:
                    //As long as we're driving, we're moving the instantaneousWaypoint's coordinate
                    //using the three velocities and the amount of time that's elasped since this move
                    //started.

                    //Grab the time now
                    double drivingCurrentTime = DSPTimeBased.getNowSeconds();

                    //Figure out the elapsed time
                    double drivingElapsedTime = drivingCurrentTime - movePlannedStartTime;

                    //Compute the next instantaneous position
                    Coordinate tempInstCoord = new Coordinate(Coordinate.CoordinateType.TerabyteF,
                            xVel * drivingElapsedTime + startOfDriveCoordinate.getX(),
                            yVel * drivingElapsedTime + startOfDriveCoordinate.getY(),
                            rotVel * drivingElapsedTime + startOfDriveCoordinate.getThetaRAD());

                    //See if we're done with this move: we're basing this entirely on elapsed time
                    if(drivingElapsedTime >= movePlannedTimeDelta)
                    {
                        //We're done.  Go back to idle
                        state = States.Idle;

                        //Make sure we didn't over drive the end point by just using the end coord.
                        tempInstCoord = new Coordinate(targetWaypoint.getCoord());

                        //See if we were waiting to notify of reaching this waypoint
                        if(targetWaypoint.getId() == this.waypointIDForTrigger)
                        {
                            this.waypointTriggered = true;
                        }
                    }

                    instantaneousCoordinate = tempInstCoord;
                    break;
            }
        }

        //Log some stuff
        if(logger != null)
        {
            double Tx =0.0;
            double Ty =0.0;
            double Tth =0.0;
            int id = -9999;
            if(targetWaypoint != null)
            {
                Tx = targetWaypoint.getCoord().getX();
                Ty = targetWaypoint.getCoord().getY();
                Tth = targetWaypoint.getCoord().getThetaRAD();
                id = targetWaypoint.getId();
            }
            logger.addValue(Tx, "RD-Tx");
            logger.addValue(Ty, "RD-Ty");
            logger.addValue(Tth, "RD-Tth");
            logger.addValue(id, "RD-Tid");

            double Ix=0.0;
            double Iy=0.0;
            double Ith=0.0;
            if(instantaneousCoordinate != null)
            {
                Ix = instantaneousCoordinate.getX();
                Iy = instantaneousCoordinate.getY();
                Ith = instantaneousCoordinate.getThetaRAD();
            }
            logger.addValue(Ix, "RD-Ix");
            logger.addValue(Iy, "RD-Iy");
            logger.addValue(Ith, "RD-Ith");
            logger.addValue(waypointQueue.size(), "RD-Qsz");
        }
    }

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
