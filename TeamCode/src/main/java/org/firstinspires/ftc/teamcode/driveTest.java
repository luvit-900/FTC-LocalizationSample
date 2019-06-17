package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.terabytesrobotics.Coordinate;
import com.terabytesrobotics.DSPTimeBased;
import com.terabytesrobotics.FieldTransform;
import com.terabytesrobotics.LocalizationSaveState;
import com.terabytesrobotics.RobotDriver;
import com.terabytesrobotics.SBSDataLogger;
import com.terabytesrobotics.SBSDriveMotion;
import com.terabytesrobotics.SBSIMU;
import com.terabytesrobotics.SBSVision;

@TeleOp(name="driveTest", group="Opmode")
public class driveTest extends OpMode {
    SBSDataLogger DataLogger;
    SBSDriveMotion DriveMotion;
    SBSIMU IMU;
    SBSVision Vision;

    @Override
    public void init()
    {
        RobotDefinitions.refreshCalValues();
        DataLogger = new SBSDataLogger("DT");
        Vision = new SBSVision(hardwareMap);
        IMU = new SBSIMU(hardwareMap.get(BNO055IMU.class, RobotDefinitions.IMU1),
                false,
                "driveTest",
                0.0,        //zeroes IMU, assumed Xr || Xf (and Yr || Yf) orientation
                LocalizationSaveState.getAutonOpModeColor(),
                DataLogger);
        DriveMotion = new SBSDriveMotion((DcMotorEx) hardwareMap.get(RobotDefinitions.wheelOneMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelTwoMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelThreeMotor),
                (DcMotorEx) hardwareMap.get(RobotDefinitions.wheelFourMotor),
                DataLogger,
                IMU);

        Vision.init();
        DriveMotion.init();
        IMU.init();

        //set initial coordinate to origin
        try {
            DriveMotion.getReckoner().SetHolonomicPosition(new Coordinate(Coordinate.CoordinateType.TerabyteF, 0.0, 0.0, 0.0)/*LocalizationSaveState.getLastLocalizationPos()*/);
        } catch (Exception e) {

        }

    }
    @Override
    public void start()
    {
        DSPTimeBased.restart();
        DataLogger.start();
        Vision.start();
        DriveMotion.start();
        IMU.start();
    }
    @Override
    public void loop()
    {
        DSPTimeBased.updateTimestamp();

        Vision.loop();
        IMU.loop();

        //get gamepad values
        double driveX = gamepad1.left_stick_x;
        double driveY = -gamepad1.left_stick_y; //Controller Y is inverted (up is negative by default [???] so we need to negate this value to make this accurate)

        //send gamepad values to drive code
        DriveMotion.cartesianInput(driveX, driveY);
        DriveMotion.setR(gamepad1.right_stick_x);

        //Execute drive code
        DriveMotion.loop();

        //Vuforia position update
        if(Vision.isTargetVisible())
        {
            DriveMotion.getReckoner().SetHolonomicPosition(FieldTransform.toTerabyte(Vision.getCoordiante()).getX(), FieldTransform.toTerabyte(Vision.getCoordiante()).getY());
            //When the IMU is used as the override angle, there are errors when rotating and driving simultaneously.
            //Hence the angle needs to be updated by the IMU periodically.
            DriveMotion.getReckoner().SetHolonomicPosition(Math.toRadians(IMU.getYaw()));
        }

        telemetry.addData("DSP-Time: ", DSPTimeBased.getNowSeconds());
        telemetry.addData("Yaw", IMU.getYaw());
        telemetry.addData("RekPos: ", DriveMotion.getReckoner().GetHolonomicPosition().format());  //Displays the robot's field position from the dead reckoner.
        telemetry.addData("VuforiaTargetVisible: ", Vision.isTargetVisible());
        telemetry.update();
        //Datalogger's loop() is last of all loop() work
        DataLogger.loop();
    }
    @Override
    public void stop()
    {
        IMU.stop();
        Vision.stop();
        DriveMotion.stop();
        DataLogger.stop();
    }
}
