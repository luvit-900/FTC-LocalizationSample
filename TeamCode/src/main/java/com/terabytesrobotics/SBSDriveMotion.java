package com.terabytesrobotics;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotDefinitions;

import static java.lang.Math.abs;

//import org.firstinspires.ftc.teamcode.opmodes.DriveTrainControl;



/**
 * Created by Ronald on 10/6/2017.
 */

public class SBSDriveMotion
{

    // Input for the drive coordinates.
    double x = 0;
    double y =  0;
    double r = 0;
    private double heading;

    double Theta = 0;
    double Mag = 0;
    double lastMag = 0;
    double lastTheta;
    double accel = 0;
    double deltaMag;

    final double acceptableAccel = 1;

    //Create instances of DcMotors. They will hold the inputs for the wheel ports.

    DcMotor wheelOne;
    DcMotor wheelTwo;
    DcMotor wheelThree;
    DcMotor wheelFour;

    //Objct-Local storage for current motor positions.  THese are updated in init() and loop() methods.
    int wheelOnePos;
    int wheelTwoPos;
    int wheelThreePos;
    int wheelFourPos;

    private int lastWheelOnePos = 0;
    private int lastWheelTwoPos = 0;
    private int lastWheelThreePos = 0;
    private int lastWheelFourPos = 0;

    //This will be our reference to the data logger.
    SBSDataLogger ourLogger;

    // The speeds array will store the speed setpoint for each wheel.
    private double[] speeds;

    //Deadreckoner keeps track of where we are on the field, based on our encoder
    //changes.
    HolonomicDeadReckon reckoner;

    // Constructor for this class. The method will store the ports for each motor.

    SBSIMU imu;

    public SBSDriveMotion(DcMotor wheelOne,
                          DcMotor wheelTwo,
                          DcMotor wheelThree,
                          DcMotor wheelFour,
                          SBSDataLogger logger,
                          SBSIMU imu){
        this.wheelOne = wheelOne;
        this.wheelTwo = wheelTwo;
        this.wheelThree = wheelThree;
        this.wheelFour = wheelFour;
        this.ourLogger = logger;
        this.imu = imu;

        //Instantiate the reckoner
        reckoner = new HolonomicDeadReckon(RobotDefinitions.driveWheelDistancePerCount,
                                            RobotDefinitions.robotWheelCircleCircumfrence);
    }


    public void init()
    {

        //Set runMode to RUN_USING_ENCODERS, which activates the robot's built-in speed control
        wheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFour.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Snag the current encoder positions
        wheelOnePos = wheelOne.getCurrentPosition();
        wheelTwoPos = wheelTwo.getCurrentPosition();
        wheelThreePos = wheelThree.getCurrentPosition();
        wheelFourPos = wheelFour.getCurrentPosition();

        //Tell the reckoner where the encoder counts are at Init in case something else
        //does something with us between init() and first call to loop()
        reckoner.SetMotorEncoders(wheelOnePos,
                wheelTwoPos,
                wheelThreePos,
                wheelFourPos);
    }

    public void start()
    {
    }

    public void loop() {

        reckoner.SetHolonomicAngleOverride(Math.toRadians(imu.getYaw()), false);

        //Log our inputs for motion, presuming there's a logger
        if (ourLogger != null) {
            ourLogger.addValue(Mag, "Mot-Mag");
            ourLogger.addValue(Theta, "Mot-Theta");
            ourLogger.addValue(r, "Mot-r");
        }

        //Compute the motor speed setpoint vector
        speeds = HolonomicMath.controlsToSpeed(Mag, Theta, r);

        //Log the outputs from the motor speed setpoints, presuming there's a logger
        if (ourLogger != null) {
            ourLogger.addValue(speeds[0], "Mot-Ssp0");
            ourLogger.addValue(speeds[1], "Mot-Ssp1");
            ourLogger.addValue(speeds[2], "Mot-Ssp2");
            ourLogger.addValue(speeds[3], "Mot-Ssp3");
        }

        //Send the motor speeds to the drive wheels.
        wheelOne.setPower(speeds[0]);
        wheelTwo.setPower(speeds[1]);
        wheelThree.setPower(speeds[2]);
        wheelFour.setPower(speeds[3]);


        //Snag the current encoder positions
        lastWheelOnePos = wheelOnePos;
        lastWheelTwoPos = wheelTwoPos;
        lastWheelThreePos = wheelThreePos;
        lastWheelFourPos = wheelFourPos;

        wheelOnePos = wheelOne.getCurrentPosition();
        wheelTwoPos = wheelTwo.getCurrentPosition();
        wheelThreePos = wheelThree.getCurrentPosition();
        wheelFourPos = wheelFour.getCurrentPosition();

        //Update the reckoner with new encoder positions
        reckoner.UpdateMotorEncoders(wheelOnePos,
                wheelTwoPos,
                wheelThreePos,
                wheelFourPos);

        //While we're here, log the current encoder positions for all of the motors
        if(ourLogger != null)
        {
            //Log raw encoder postions
            ourLogger.addValue(wheelOnePos,   "Mot-Pos0");
            ourLogger.addValue(wheelTwoPos,   "Mot-Pos1");
            ourLogger.addValue(wheelThreePos, "Mot-Pos2");
            ourLogger.addValue(wheelFourPos,  "Mot-Pos3");

            //Log what the reckoner thinks our field position is
            Coordinate roboPos = reckoner.GetHolonomicPosition();
            ourLogger.addValue(roboPos.getThetaRAD(), "Rek-Theta");
            ourLogger.addValue(roboPos.getX(), "Rek-Xf");
            ourLogger.addValue(roboPos.getY(), "Rek-Yf");
        }


        //Check against a weird crash issue
        int maxPossibleEncDelta = (int)(3080.0 * DSPTimeBased.getDeltaT());
        if(Math.abs(lastWheelOnePos - wheelOnePos) > maxPossibleEncDelta ||
                Math.abs(lastWheelTwoPos - wheelTwoPos) > maxPossibleEncDelta ||
                Math.abs(lastWheelThreePos - wheelThreePos) > maxPossibleEncDelta ||
                Math.abs(lastWheelFourPos - wheelFourPos) > maxPossibleEncDelta)
        {
            //if the encoders changed by more than is possible in a given time period, don't log position.
            // motor speed (no gearbox) = 6600rpm;  encoder counts per rotation = 28;  3080 max delta counts per second.
            //(if the program crashes, sometimes the REV Hubs report an all zero reading on the sensors and encoders.  If this happens, the following code prevents us from logging invalid values.)
        }
        else
        {
            LocalizationSaveState.saveRobotFieldLocation(reckoner.GetHolonomicPosition());
        }

    }



    public void stop()
    {

    }

    /*
    Returns a reference to this SBS Drive motion's Dead Reckoner.
    This allows another class to set or update initial position and
    to access the reckoned position.
     */
    public HolonomicDeadReckon getReckoner()
    {
        return reckoner;
    }

    //These methods set the x,y,and r variables for driving
    public void cartesianInput(double x, double y)
    {
        Theta = Math.atan2(y, x);

        lastMag = this.Mag;

        Mag = (Math.sqrt(x*x + y*y));

    }
    public void polarInput(double Mag, double theta)
    {
        lastMag = this.Mag;
        this.Mag = Mag;
        this.Theta = theta;
    }

    public void setR(double input)
    {
        r = input;
    }
}
