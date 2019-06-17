package com.terabytesrobotics;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Ronald on 10/6/2017.
 */

public class SBSVision
{
    private static final String VUFORIA_KEY = "AQAnyBv/////AAAAGa0435XOy0wou5kkQss8MohxYP0/2xhRTQeV5+sun2Qo2CTrlJis9/FgoPWH2vNNFx5dgJMXG9hwO1g02w8dhRMRi/toh9/jZXiNDoiAWLdolI/ueK4wRu3DT2VJEESeBqKdwvAlzMD/hKd8lthly9yMJoOCOhJebniK7norRWHjF5cTxOjwfhmKfTeC7vdmSI/8etc7zorq9Z5E5yT/Bu9p3JweuduY8NaeMSP7zW3ZjQsdhp9vyxF0Rm55NUxgR94QdPXulGFu+m06pJr1m/DRfOwZu50sFyEMxh0V9yN6coQCOF66YCFdXPqeyDSWFPB0qUuRHWFAxfODTwQHhwmFenzFiLyqflieLbFns42Q";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    //Location of Camera relative on the robot (see below for description)
    /**
     * Create a transformation matrix describing where the phone is on the robot.
     *
     * The coordinate frame for the robot looks the same as the field.
     * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
     * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
     *
     * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
     * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
     * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
     *
     * If using the rear (High Res) camera:
     * We need to rotate the camera around it's long axis to bring the rear camera forward.
     * This requires a negative 90 degree rotation on the Y axis
     *
     * If using the Front (Low Res) camera
     * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
     * This requires a Positive 90 degree rotation on the Y axis
     *
     * Next, translate the camera lens to where it is on the robot.
     * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
     */
    final int CAMERA_FORWARD_DISPLACEMENT  = 200;   // eg: Camera is 200 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 40;   // eg: Camera is 40 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 200; // eg: Camera is 200 mm to the right of the robot's center line
    final double CAMERA_DIRECTION = 0.0;

    //TRAINING DATA for Vuforia
    VuforiaTrackables targetsRoverRuckus;

    //Array List to hold all of the Vuforia targets
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Hardware map to get camera
    HardwareMap hardwareMap;

    //ROBOT LOCATION and ORIENTATION variables
    private double X = 0;
    private double Y = 0;
    private double Z = 0;
    private double heading = 0;
    private double roll = 0;
    private double pitch = 0;

    //Status Variables
    private boolean targetVisible = false;
    private String targetName = "";

    public SBSVision(HardwareMap HWM)
    {
        this.hardwareMap = HWM;
    }

    public void init()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, (float)CAMERA_DIRECTION));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    public void start()
    {
        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public void loop()
    {
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetName = trackable.getName();
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            X = translation.get(0) / mmPerInch;
            Y = translation.get(1) / mmPerInch;
            Z = translation.get(2) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            roll = rotation.firstAngle;
            pitch = rotation.secondAngle;
            heading = rotation.thirdAngle;
        }
        else {
            X = 0;
            Y = 0;
            Z = 0;
            roll = 0;
            pitch = 0;
            heading = 0;
        }

    }

    public void stop()
    {

    }

    public double getX()
    {
        return X;
    }

    public double getY()
    {
        return Y;
    }

    public double getZ()
    {
        return Z;
    }

    public double getHeading()
    {
        return Math.toRadians(heading);
    }

    public Coordinate getCoordiante()
    {
        return new Coordinate(Coordinate.CoordinateType.VuforiaF, X, Y, Math.toRadians(heading));
    }

    public boolean isTargetVisible()
    {
        return targetVisible;
    }

    public String getTargetName()
    {
        return targetName;
    }
}
