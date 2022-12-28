package com.hadronknights.ftc.robot.command;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.subsystem.HkVuforia;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DetectGoldMineralCommand extends BaseCommand {

    private HkVuforia hkVuforia;
    private Drivetrain drivetrain;
    private HkPosition goldMineralPosition;

    public DetectGoldMineralCommand(LinearOpMode opMode, HkRobot robot, HkVuforia hkVuforia) {
        super(opMode);
        this.drivetrain = robot.getDrivetrain();
        this.hkVuforia = hkVuforia;
    }

    @Override
    public void run() {
        hkVuforia.activateTracking();
        GoldAlignDetector detector = new GoldAlignDetector();
        detector.init(opMode.hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA

        hkVuforia.startDetection(detector);

        double incrementalDistance = 1;
        double totalDistance = 0;

        while(opMode.opModeIsActive() && !detector.isFound() && totalDistance < 35){
            drivetrain.moveForward(0.4, incrementalDistance);
            totalDistance += incrementalDistance;
            sleepWithActiveCheck(2);
        }

        telemetry.addData("Is Found Gold: ", detector.isFound());
        telemetry.update();

        if (totalDistance < 7) {
            goldMineralPosition = HkPosition.RIGHT;
        }
        else if (totalDistance < 21) {
            goldMineralPosition = HkPosition.CENTER;
        }
        else {
            goldMineralPosition = HkPosition.LEFT;
        }
    }

    public HkPosition getGoldMineralPosition() {
        return goldMineralPosition;
    }
}
