package com.hadronknights.ftc.opmode.autonomous.test;
import com.hadronknights.ftc.opmode.autonomous.BaseAutonomous;

import com.hadronknights.ftc.robot.command.LandCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Square On Line Command", group ="HKTest")

public class LandCommandTest extends BaseAutonomous {

    @Override
    public void runAutonomous() {

        Runnable cmd = new LandCommand(this, robot);
        cmd.run();
    }
}
