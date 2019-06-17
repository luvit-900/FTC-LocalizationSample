package com.terabytesrobotics;

import android.os.Environment;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.io.File;
import java.io.FileInputStream;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

public class xmlReader {
    //reads a double from a .xml file (returns 1.0 by default)
    public static double readXML(String xml_File, String tag) {
        double value = 1.0;  //default value.
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
                if(Double.isInfinite(value))
                {
                    value = 1.0;  //default value
                }
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
