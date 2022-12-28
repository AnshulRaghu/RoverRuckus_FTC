package com.hadronknights.ftc.opmode.autonomous.test;

import com.hadronknights.ftc.robot.subsystem.Lift;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Lift Subsystem", group ="HKTest")
public class LiftTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        telemetry.addData("Initialization", "HK Robot created and initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Waiting for Play", "Wait for Referees and then Press play");
            telemetry.update();
            idle();
        }

        waitForStart();
        lift.moveLiftUp();
        sleep(2000);
        lift.moveLiftDown();
    }
}
