package com.hadronknights.ftc.robot.command;

import com.hadronknights.ftc.robot.HkColor;
import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ClaimMarkerLeftCommand extends BaseCommand {

    private HkRobot robot;
    private Drivetrain drivetrain;
    private HkColor allianceColor;
    private HkPosition goldMineralPosition;

    public ClaimMarkerLeftCommand(LinearOpMode opMode, HkRobot robot, HkColor allianceColor, HkPosition goldMineralPosition) {
        super(opMode);
        this.robot = robot;
        this.drivetrain = robot.getDrivetrain();
        this.allianceColor = allianceColor;
        this.goldMineralPosition = goldMineralPosition;
    }

    @Override
    public void run() {
        switch (goldMineralPosition) {
            case RIGHT:
                drivetrain.moveForward(0.4, 36);
                break;
            case CENTER:
                drivetrain.moveForward(0.4, 21.5);
                break;
            case LEFT:
            default:
                drivetrain.moveForward(0.4, 7);
                break;
        }
        sleepWithActiveCheck(400);

        drivetrain.turnRight(0.5, 43);
        sleepWithActiveCheck(50);

        MoveUntilWallCommand wallCommand = new MoveUntilWallCommand(opMode, robot, robot.getDistanceSensor(), 8, 0.3);
        wallCommand.run();
        sleepWithActiveCheck(400);

        drivetrain.turnLeft(0.5, 93);
        sleepWithActiveCheck(50);

        drivetrain.moveBackward(0.5, 30);
        sleepWithActiveCheck(10);

        MoveUntilLineCommand lineCommand = new MoveUntilLineCommand(opMode, robot, allianceColor, robot. getLeftColorSensor(), 0.2);
        lineCommand.run();
        sleepWithActiveCheck(10);

        drivetrain.moveBackward(0.4, 5);
        sleepWithActiveCheck(20);

        if (robot.getTransporter() != null) {
            robot.getTransporter().releaseMarker();
            sleepWithActiveCheck(50);
            drivetrain.moveForward(5, 0.5);
            sleepWithActiveCheck(10);
            robot.getTransporter().pickUpArm();
        }
    }
}
