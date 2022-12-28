package com.hadronknights.ftc.robot;

import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.drivetrain.TwoMotorSimpleDrivetrain;
import com.hadronknights.ftc.robot.subsystem.Lift;
import com.hadronknights.ftc.robot.subsystem.MarkerTransporter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * This class represents the robot of Hadron Knights.
 * <p>
 * Created by Rahul V, Arko, Rahul D, Aarush on 8/19/2017.
 */

public class HkRobot
{
    /**
     * Drive train of the robot
     */
    private Drivetrain drivetrain;

    /**
     * Left Color sensor
     */
    private ColorSensor leftColorSensor;

    /**
     * Right Color sensor
     */
    private ColorSensor rightColorSensor;

    /**
     * Side Color sensor
     */
    private ColorSensor sideColorSensor;

    /**
     * Distance sensor
     */
    private DistanceSensor distanceSensor;

    private Lift lift;

    private MarkerTransporter transporter;


     /**
     * Constructor.
     *
     * @param opMode
     * @param isAutonomous
     */
    public HkRobot(OpMode opMode, boolean isAutonomous)
    {
        drivetrain = new TwoMotorSimpleDrivetrain(opMode, isAutonomous);

        leftColorSensor = opMode.hardwareMap.get(ColorSensor.class, "LeftColorSensor");
        rightColorSensor = opMode.hardwareMap.get(ColorSensor.class, "RightColorSensor");
        //sideColorSensor = opMode.hardwareMap.get(ColorSensor.class, "SideColorSensor");
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        lift = new Lift (opMode);
        transporter = new MarkerTransporter(opMode);
    }
    

    /**
     * Returns the drive train for the robot.
     *
     * @return Drivetrain
     */
    public Drivetrain getDrivetrain()
    {
        return drivetrain;
    }

    public ColorSensor getLeftColorSensor() { return leftColorSensor; }

    public ColorSensor getRightColorSensor() { return rightColorSensor; }

    public ColorSensor getSideColorSensor() { return sideColorSensor; }

    public DistanceSensor getDistanceSensor() { return distanceSensor; }

    public Lift getLift () { return lift; }

    public MarkerTransporter getTransporter () { return transporter; }
}
