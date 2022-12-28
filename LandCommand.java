package com.hadronknights.ftc.robot.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Lift;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LandCommand  extends BaseCommand{

    private Lift lift;

    public LandCommand (LinearOpMode opMode, HkRobot robot) {
        super(opMode);
        this.lift = robot.getLift();
    }

    public void run ()
    {
        lift.moveLiftUp();
        opMode.sleep(50);
        lift.openHook();
        opMode.sleep(50);
        lift.moveLiftDown();
        opMode.sleep(50);
        lift.closeHook();
        opMode.sleep(50);
    }

}
