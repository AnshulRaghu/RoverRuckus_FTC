package com.hadronknights.ftc.opmode.teleop;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Lift;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This class controls the Hadron Knights robot during the driver controlled part of the game.
 * <p>
 * Created by Rahul V, Arko, Rahul D, Aarush on 8/19/2017.
 */
@TeleOp(name = "TeleOp", group = "HKTeleOp")

public class Teleop extends OpMode
{

    private HkRobot robot;
    
    /**
     * OpMode constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    public Teleop()
    {
        super();
    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init()
    {
        robot = new HkRobot(this, false);
        telemetry.addData("Initilization", "Initialized teleop");
        telemetry.update();
    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    @Override
    public void init_loop()
    {
        super.init_loop();
    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    @Override
    public void start()
    {
        super.start();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop()
    {
        telemetry.addData("Loop", "Starting teleop loop");
        telemetry.update();

        Lift lift = robot.getLift();

        robot.getDrivetrain().drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        if(lift != null)
        {
           lift.moveLift(gamepad1.right_bumper, gamepad1.left_bumper);
           lift.controlHook(gamepad1.right_trigger, gamepad1.left_trigger);
        }
    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    @Override
    public void stop()
    {
        super.stop();
    }
}
