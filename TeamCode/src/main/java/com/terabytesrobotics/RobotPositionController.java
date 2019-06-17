package com.terabytesrobotics;

public class RobotPositionController
{
    //Controllers.
    PIcontroller xController;
    PIcontroller yController;
    PIcontroller rotController;

    //Errors, in field frame or reference
    double xFieldError;
    double yFieldError;
    double rotFieldError;

    //Field frame velocities
    double xFieldVel;
    double yFieldVel;
    double rotFieldVel;

    //Computed mag, theta and rot velocity commands
    double linearVelMagnitude;
    double linearVelTheta;
    double rotaionalVelMagnitude;

    Coordinate fieldFeedbackPosition;
    Coordinate fieldTargetPosition;

    //Limits on when we say we're there
    double linearTolerance=0.0;
    double rotTolerance=0.0;

    SBSDataLogger logger;

    //Constructor with a logger reference
    public RobotPositionController(SBSDataLogger logger)
    {
        xController = new PIcontroller();
        yController = new PIcontroller();
        rotController = new PIcontroller();

        //All of the output limits can be the same +/-1.0, just like teleop expects
        xController.setOutputLimit(1.0, -1.0);
        yController.setOutputLimit(1.0, -1.0);
        rotController.setOutputLimit(1.0, -1.0);

        //Save our logger reference, if any
        this.logger = logger;
    }

    //Constructor with no logger reference
    public RobotPositionController()
    {
        this(null);
    }

    //Takes field position of robot Cooridinate, TerabyteF, angle representing angle of robot
    //takes field target position for robot Coordinate, TerabyteF, angle representing desired angle of robot
    //Computes a robot relative Mag & Theta of linear drive speed command and a rotational command

    //
    // Setters for Inputs
    //
    //Input of the current field position
    public void setRobotFeedbackPosition(Coordinate fieldFeedbackPosition)
    {
        this.fieldFeedbackPosition = new Coordinate(fieldFeedbackPosition);
    }

    //Input of the target field position
    public void setRobotTargetPosition(Coordinate fieldTargetPosition)
    {
        this.fieldTargetPosition = new Coordinate(fieldTargetPosition);
    }

    //
    // Compute the controller outputs
    //
    public void compute()
    {
        //We need to pre-compute the error value for the rotational axis because it's strange and loops around.
        rotFieldError = (fieldTargetPosition.getThetaRAD() - fieldFeedbackPosition.getThetaRAD() + 20.0 * Math.PI) % (2.0 * Math.PI);
        if(rotFieldError > Math.PI)
        {
            rotFieldError -= 2.0*Math.PI;
        }

        //Compute the controllers for the three degrees of freedom, in field frame reference.
        xFieldVel = this.xController.processInput(fieldTargetPosition.getX(), fieldFeedbackPosition.getX());
        yFieldVel = this.yController.processInput(fieldTargetPosition.getY(), fieldFeedbackPosition.getY());
        rotFieldVel = this.rotController.processInput(rotFieldError, 0.0);  //Use the rotational error in place

        //Get the error terms back because we need them for other checking
        xFieldError = this.xController.getError();
        yFieldError = this.yController.getError();

        //Move these velocities onto the robot frame of reference
        //-Magnitude is the same in field frame as in robot frame
        linearVelMagnitude = Math.sqrt(xFieldVel * xFieldVel + yFieldVel * yFieldVel);
        //-Find the angle of the velocity command vector and subract the robot angle
        linearVelTheta = Math.atan2(yFieldVel, xFieldVel) - fieldFeedbackPosition.getThetaRAD();
        //Rotational is rotational
        rotaionalVelMagnitude = rotFieldVel;

        //Log stuff, if we have a logger
        if(logger != null)
        {
            //Errors
            logger.addValue(xFieldError, "PC-Ex");
            logger.addValue(yFieldError, "PC-Ey");
            logger.addValue(rotFieldError, "PC-Erot");
            //Field-frame velocity commands
            logger.addValue(xFieldVel, "PC-Vx");
            logger.addValue(yFieldVel, "PC-Vy");
            logger.addValue(rotFieldVel, "PC-Vr");
            //robot frame polar drive velocity commands
            logger.addValue(linearVelMagnitude, "PC-LvM");
            logger.addValue(linearVelTheta, "PC-LvTh");
            logger.addValue(rotaionalVelMagnitude, "PC-RvM");
        }
    }

    //
    // Getters for drive parameters
    //
    public double getLinVelMag()
    {
        return linearVelMagnitude;
    }
    public double getLinVelTheta()
    {
        return linearVelTheta;
    }
    public double getRotVel()
    {
        return rotaionalVelMagnitude;
    }
    //
    // Getter for the errors being withing limit
    //
    public boolean getPositionInControl()
    {
        boolean bXInControl = Math.abs(this.xFieldError) < this.linearTolerance;
        boolean bYInControl = Math.abs(this.yFieldError) < this.linearTolerance;
        boolean bRotInControl = Math.abs(this.rotFieldError) < this.rotTolerance;

        return bXInControl && bYInControl && bRotInControl;
    }


    //
    // Setters for parameters
    //
    //Setter for X gain
    public void setKpGainX(double x)
    {
        this.xController.setKp(x);
    }
    public void setKiGainX(double x)
    {
        this.xController.setKi(x);
    }
    //Setter for Y gain
    public void setKpGainY(double y)
    {
        this.yController.setKp(y);
    }
    public void setKiGainY(double y)
    {
        this.yController.setKi(y);
    }
    //Setter for Rotational gain
    public void setKpGainRot(double rot)
    {
        this.rotController.setKp(rot);
    }
    public void setKiGainRot(double rot)
    {
        this.rotController.setKi(rot);
    }
    //Setter for X, Y (linear) "in-control" limit
    public void setLinearControlTolerance(double linTol)
    {
        this.linearTolerance = linTol;
    }
    //Setter for rotational "in-control" limit
    public void setRotationalControlTolerance(double rotTol)
    {
        this.rotTolerance = rotTol;
    }
}
