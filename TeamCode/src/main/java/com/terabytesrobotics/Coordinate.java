package com.terabytesrobotics;

import java.text.DecimalFormat;

public class Coordinate {

    //X and Y are ALWAYS in inches!!!!!!!!!!!!!!!

    public enum CoordinateType
    {
        TerabyteF,  // coordinates for Terabyte field
        VuforiaF,  // coordinates for Vuforia Field
        Robot,  // coordinates for Robot frame of reference
        UNKNOWN
    }

    protected CoordinateType m_type = CoordinateType.UNKNOWN;

    protected double m_x;

    protected double m_y;

    protected double m_theta; //IN RADIANS!!!!!!!!!!!!!!

    public Coordinate(CoordinateType type, double x, double y, double thetaRAD)
    {
        m_type = type;
        m_x = x;
        m_y = y;
        m_theta = thetaRAD;
    }

    //Copy constructor
    public Coordinate(Coordinate source)
    {
        this.m_type = source.m_type;
        this.m_x = source.m_x;
        this.m_y = source.m_y;
        this.m_theta = source.m_theta;
    }

    public String format()
    {
        DecimalFormat format = new DecimalFormat("#.###");
        return "(" + format.format(m_x) + ", " + format.format(m_y) + ", " + format.format(m_theta) + ", " + m_type + ")";
    }

    public double getX()
    {
        return  m_x;
    }

    public double getY()
    {
        return m_y;
    }

    public double getThetaRAD()
    {
        return m_theta;
    }

    public CoordinateType getType()
    {
        return m_type;
    }

    public boolean isSameType(Coordinate suspect)
    {
        return (suspect.m_type == this.m_type);
    }

    public boolean isSameType(Coordinate.CoordinateType suspect_type)
    {
        return (suspect_type == this.m_type);
    }
}
