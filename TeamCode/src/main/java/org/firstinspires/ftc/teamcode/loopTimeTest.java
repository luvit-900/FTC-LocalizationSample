package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.terabytesrobotics.DSPTimeBased;
import com.terabytesrobotics.SBSDataLogger;

@TeleOp(name="loopTimeTest", group="Opmode")
public class loopTimeTest extends OpMode {

    SBSDataLogger DataLogger;

    //The phone plugs into hub 2

    @Override
    public void init()
    {
        DataLogger = new SBSDataLogger("LT");
    }
    @Override
    public void start()
    {
        DSPTimeBased.restart();
        DataLogger.start();
    }
    @Override
    public void loop()
    {
        DSPTimeBased.updateTimestamp();

        DataLogger.addValue(gamepad1.a, "a");
        DataLogger.addValue(gamepad1.b, "b");
        DataLogger.addValue(gamepad1.x, "x");
        DataLogger.addValue(gamepad1.y, "y");

        DataLogger.loop();
    }
    @Override
    public void stop()
    {
        DataLogger.stop();
    }
}
