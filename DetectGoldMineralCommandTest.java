package com.hadronknights.ftc.opmode.autonomous.test;
import com.hadronknights.ftc.opmode.autonomous.BaseAutonomous;

import com.hadronknights.ftc.robot.command.DetectGoldMineralCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Detect Gold Mineral Command", group ="HKTest")
public class DetectGoldMineralCommandTest extends BaseAutonomous {

    @Override
    public void runAutonomous() {

        DetectGoldMineralCommand cmd = new DetectGoldMineralCommand(this, robot, hkVuforia);
        cmd.run();
    }
}
