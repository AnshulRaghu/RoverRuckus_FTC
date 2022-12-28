package com.hadronknights.ftc.robot.command;

import android.graphics.Color;

import com.hadronknights.ftc.robot.HkColor;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class SquareOnLineCommand extends BaseCommand
{
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private final double SCALE_FACTOR = 255;
    private Drivetrain drivetrain;
    private HkColor lineColor;

    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    public SquareOnLineCommand(LinearOpMode opMode, HkRobot robot, HkColor lineColor)
    {
        super(opMode);
        this.leftColorSensor = robot.getLeftColorSensor();
        this.rightColorSensor = robot.getRightColorSensor();
        this.telemetry = opMode.telemetry;
        this.lineColor = lineColor;
    }

    @Override
    public void run()
    {

        while (opMode.opModeIsActive() && readColor(leftColorSensor) != lineColor && readColor(rightColorSensor) != lineColor)
        {
            // both left and right color sensor is not on the line
            drivetrain.setPower(0.3, 0, 0.3, 0);
        }
        drivetrain.stop();

        sleepWithActiveCheck(10);

        if (opMode.opModeIsActive() && readColor(leftColorSensor) != lineColor)
        {
            // left color sensor is not on the line
            while (opMode.opModeIsActive() && readColor(leftColorSensor) != lineColor)
            {
                drivetrain.setPower(0.5, 0, -0.5, 0);
            }
            drivetrain.stop();
        } else if (opMode.opModeIsActive() && readColor(rightColorSensor) != lineColor)
        {
            // right color sensor is not on the line
            while (opMode.opModeIsActive() && readColor(rightColorSensor) != lineColor)
            {
                drivetrain.setPower(-0.5, 0, 0.5, 0);
            }
            drivetrain.stop();
        }
    }


    private HkColor readColor(ColorSensor colorSensor)
    {
        double targetBlueHue = 205;
        double targetRedHue = 350;
        double hue = getHue(colorSensor);

        if (hue > (targetBlueHue - 25) && hue < (targetBlueHue + 25))
        {
            telemetry.addData("Color Sensed", "Blue");
            telemetry.update();
            return HkColor.BLUE;
        } else if (hue > (targetRedHue - 25) && hue < (targetRedHue + 25))
        {
            telemetry.addData("Color Sensed", "Red");
            telemetry.update();
            return HkColor.RED;
        } else
        {
            telemetry.addData("Color Sensed", "Color Not Sensed");
            telemetry.update();
            return HkColor.UNKNOWN;
        }
    }

    private float getHue(ColorSensor colorSensor)
    {
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
