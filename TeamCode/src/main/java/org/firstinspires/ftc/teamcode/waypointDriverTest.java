package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.terabytesrobotics.Coordinate;
import com.terabytesrobotics.DSPTimeBased;
import com.terabytesrobotics.RobotDriver;
import com.terabytesrobotics.RobotPositionController;
import com.terabytesrobotics.SBSDataLogger;
import com.terabytesrobotics.SBSDriveMotion;
import com.terabytesrobotics.SBSIMU;
import com.terabytesrobotics.Waypoint;

@Autonomous(name="waypointDriverTest", group="Opmode")
public class waypointDriverTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        SBSDataLogger DataLogger;
        SBSDriveMotion DriveMotion;
        SBSIMU IMU;
        RobotDriver arde;   //Pronounced "ar-dee", this driver's name
        RobotPositionController posCtrlr;

        //Subsystem constructors
        DataLogger = new SBSDataLogger("WT");
        arde = new RobotDriver(DataLogger); //Instantiate the robot driver
        IMU = new SBSIMU(hardwareMap.get(BNO055IMU.class, RobotDefinitions.IMU1),
                true,
                "waypointDriverTest",
                0.0,        //zeroes IMU, assumed Xr || Xf (and Yr || Yf) orientation
                RobotDefinitions.AutonOpModeColor.RED,
                DataLogger);
        DriveMotion = new SBSDriveMotion((DcMotorEx) hardwareMap.get(RobotDefinitions.wheelOneMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelTwoMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelThreeMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelFourMotor),
                DataLogger,
                IMU);
        posCtrlr = new RobotPositionController(DataLogger);

        arde.init();
        DriveMotion.init();
        IMU.init();

        //Set gains of position controller
        posCtrlr.setKpGainX(0.08);
        posCtrlr.setKiGainX(0.0);
        posCtrlr.setKpGainY(0.08);
        posCtrlr.setKiGainY(0.0);
        posCtrlr.setKpGainRot(0.5);
        posCtrlr.setKiGainRot(0.0);

        telemetry.addData("Init Complete!!", "      . . . . . . . .");
        waitForStart();

        DSPTimeBased.restart();
        arde.start();
        DataLogger.start();
        DriveMotion.start();
        IMU.start();

        //Enter some waypoints into Arde
        //These waypoints build a diamond-shaped drive around the field

        //initial position
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0,  0,   0), 0, Waypoint.TimeType.Init, 0));
        //first waypoint
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 48, 48,   0), 5, Waypoint.TimeType.FromLastWaypoint, 1));
        //Wait a few seconds to make sure we don't overshoot
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 48, 48,   0), 2, Waypoint.TimeType.FromLastWaypoint, 2));
        //second waypoint
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0, 96,   0), 5, Waypoint.TimeType.FromLastWaypoint, 3));
        //Wait a few seconds to make sure we don't overshoot
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0, 96,   0), 2, Waypoint.TimeType.FromLastWaypoint, 4));
        //third waypoint
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, -48, 48,   0), 5, Waypoint.TimeType.FromLastWaypoint, 5));
        //Wait a few seconds to make sure we don't overshoot
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, -48, 48,   0), 2, Waypoint.TimeType.FromLastWaypoint, 6));
        // final destination
        arde.addWaypoint(new Waypoint(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0, 0,   0), 5, Waypoint.TimeType.FromLastWaypoint, 7));
        //-----

        //Turn on Arde, the waypoint driver
        arde.setEnable(true);

        while(opModeIsActive())
        {
            DSPTimeBased.updateTimestamp();

            IMU.loop();

            arde.loop();

            //Pass in the data about where we want the robot to be.
            Coordinate targPoint = arde.getTargetPoint();
            posCtrlr.setRobotTargetPosition(targPoint);
            //Pass in the data about where the robot is.
            Coordinate drPoint = DriveMotion.getReckoner().GetHolonomicPosition();
            posCtrlr.setRobotFeedbackPosition(drPoint);
            //Compute the controller outputs
            posCtrlr.compute();
            /* Position controller test */
                //posCtrlr generates a positive speed for increasing angle.  Drive code moves clockwise (decreases in angle) when given a positive speed (was built this way for gamepad controls).
                DriveMotion.setR(-posCtrlr.getRotVel());
                DriveMotion.polarInput(posCtrlr.getLinVelMag(), posCtrlr.getLinVelTheta());
             /**/

            DriveMotion.loop();

            telemetry.addData("DSP-Time: ", DSPTimeBased.getNowSeconds());
            telemetry.addData("Yaw", IMU.getYaw());
            telemetry.addData("WP_ID", arde.getCurrentWaypointId());
            telemetry.addData("WP_Dn", arde.isCurrentWaypointReached());
            telemetry.update();
            //Datalogger's loop() is last of all loop() work
            DataLogger.loop();
        }

        arde.stop();
        IMU.stop();
        DriveMotion.stop();
        DataLogger.stop();
    }
}
