package com.hadronknights.ftc.robot.command;

import android.graphics.Color;

import com.hadronknights.ftc.robot.HkColor;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Command to move the robot until a color line - blue or red.
 * <p>
 * Created by Rahul V 9/21/17.
 */

public class MoveUntilLineCommand extends BaseCommand {
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    private HkRobot robot;
    private Drivetrain drivetrain;
    private ColorSensor colorSensor;
    private HkColor lineColor;
    private double power;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};


    public MoveUntilLineCommand(LinearOpMode opMode, HkRobot robot, HkColor lineColor, ColorSensor colorSensor, double power) {
        super(opMode);
        this.robot = robot;
        this.lineColor = lineColor;
        this.power = power;
        this.drivetrain = robot.getDrivetrain();
        this.colorSensor = colorSensor;
    }

    /**
     * Runs the logic for this command.
     */
    public void run() {
        float hue = 0;
        float targetHue = 0;
        if (lineColor == HkColor.BLUE) {
            targetHue = 200;
        } else if (lineColor == HkColor.RED) {
            targetHue = 350;
        }
        power = Range.clip(power, -1, 1);
        drivetrain.setPower(power, power, power, power);
        while (opMode.opModeIsActive() && (hue = getHue()) < targetHue) {
            telemetry.addData("Hue", hue);
            telemetry.update();
        }

        drivetrain.stop();

    }

    private float getHue() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();


        return hsvValues[0];
    }

}
