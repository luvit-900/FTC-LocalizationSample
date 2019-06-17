package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.terabytesrobotics.xmlReader;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.io.File;
import java.io.FileInputStream;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;


@TeleOp(name="xmlReadTest", group="Opmode")
public class xmlReadTest extends OpMode {
    private double CalVal = 0.0;

    @Override
    public void init()
    {
        RobotDefinitions.refreshCalValues();
    }
    @Override
    public void start()
    {

    }
    @Override
    public void loop()
    {
        telemetry.addData("RekCalVal: ", xmlReader.readXML(RobotDefinitions.rekCalFilename, "calVal"));
        telemetry.addData("RotCalVal: ", 1.0 / xmlReader.readXML(RobotDefinitions.rotCalFilename, "calVal"));
        telemetry.update();
    }
    @Override
    public void stop()
    {

    }

    //reads a double from a .xml file
    public static double readXML(String xml_File, String tag) {
        double value = 1.0;
        String calValString = "";
        Document dom;
        // Make an  instance of the DocumentBuilderFactory
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        try {
            // use the factory to take an instance of the document builder
            DocumentBuilder db = dbf.newDocumentBuilder();
            // parse using the builder to get the DOM mapping of the
            // XML file
            File filesDir = Environment.getExternalStorageDirectory();
            dom = db.parse(new FileInputStream(new File(filesDir, xml_File)));

            Element doc = dom.getDocumentElement();

            calValString = getTextValue(calValString, doc, tag);
            if (calValString != null) {
                if (!calValString.isEmpty())
                    value = Double.parseDouble(calValString);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        return value;
    }

    private static String getTextValue(String def, Element doc, String tag) {
        String value = def;
        NodeList nl;
        nl = doc.getElementsByTagName(tag);
        if (nl.getLength() > 0 && nl.item(0).hasChildNodes()) {
            value = nl.item(0).getFirstChild().getNodeValue();
        }
        return value;
    }


}
