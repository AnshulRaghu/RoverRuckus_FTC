package com.hadronknights.ftc.opmode.autonomous.test;
import com.hadronknights.ftc.opmode.autonomous.BaseAutonomous;
import com.hadronknights.ftc.robot.HkColor;
import com.hadronknights.ftc.robot.command.SquareOnLineCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Square On Line Command", group ="HKTest")

public class SquareOnLineCommandTest extends BaseAutonomous {

    @Override
    public void runAutonomous() {

        Runnable cmd = new SquareOnLineCommand(this, robot, HkColor.BLUE);
        cmd.run();
    }
}
