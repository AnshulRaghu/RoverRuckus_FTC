package com.hadronknights.ftc.robot.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class FourMotorSimpleDrivetrain implements Drivetrain{


    private final static double TICKS_PER_ROTATION = 1120;
    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double WHEEL_CIRCUMFERENCE_IN_INCHES = WHEEL_DIAMETER_IN_INCHES * Math.PI;
    private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_IN_INCHES;

    private OpMode opMode;
    private Telemetry telemetry;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;

    public FourMotorSimpleDrivetrain(OpMode opMode, boolean isAutonomous) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        frontLeft = opMode.hardwareMap.dcMotor.get("FrontLeft");
        backLeft = opMode.hardwareMap.dcMotor.get("BackLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("FrontRight");
        backRight = opMode.hardwareMap.dcMotor.get("BackRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft. setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (isAutonomous) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            telemetry.addData("Configuration", "IMU Found");
            telemetry.update();
        }
    }

    public void drive(double gamepad1_left_stick_x, double gamepad1_left_stick_y, double gamepad1_right_stick_x, double gamepad1_right_stick_y)
    {
        setPower(-gamepad1_left_stick_y, gamepad1_left_stick_x);
    }



    @Override
    public void moveForward(double maxPower, double distanceInInches) {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition + distanceInInches * TICKS_PER_INCH;
        double power = maxPower;
        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) < targetPosition) {

            setPower(power, power, power, power);
        }
        stop();
    }

    @Override
    public void moveBackward(double maxPower, double distanceInInches)
    {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition - distanceInInches * TICKS_PER_INCH;
        double distanceToTarget = currentPosition - targetPosition;
        double power = maxPower;
        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) > targetPosition)
        {
            distanceToTarget = currentPosition - targetPosition;
            //            if (distanceToTarget < TICKS_PER_ROTATION/2) {
            //                power = distanceToTarget * PID_COEFF;
            //                power = clip(power, MIN_POWER, maxPower);
            //            }
            setPower(-power, -power, -power, -power);
//            telemetry.addData("Current Position", currentPosition);
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Distance to Target", distanceToTarget);
//            telemetry.update();
        }
        stop();
    }

    public void moveForwardWithTimeout(double maxPower, double distanceInInches, double timeout)
    {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition + distanceInInches * TICKS_PER_INCH;
        double distanceToTarget = targetPosition - currentPosition;
        double power = maxPower;
        double startTime = opMode.time;
        double elapsedTime = 0;

        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) < targetPosition && (elapsedTime = opMode.time - startTime) < timeout)
        {
            distanceToTarget = targetPosition - currentPosition;
            //            if (distanceToTarget < TICKS_PER_ROTATION/2) {
            //                power = distanceToTarget * PID_COEFF;
            //                power = clip(power, MIN_POWER, maxPower);
            //            }
            setPower(power, power, power, power);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.update();
        }


        stop();

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Distance to Target", distanceToTarget);
        telemetry.update();
    }

    @Override
    public void turnRight(double maxPower, float angleInDegrees)
    {
        angleInDegrees = angleInDegrees % 360;
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Start Angle", startAngle);
        telemetry.update();
        float angleTurned;


        while (getLinearOpMode().opModeIsActive() && (angleTurned = findTurnClockwise(startAngle)) < angleInDegrees)
        {
            setPower(maxPower, maxPower, -maxPower, -maxPower);
            telemetry.addData("Angle Turned", angleTurned);
            telemetry.update();
        }

        stop();
    }

    @Override
    public void turnLeft(double maxPower, float angleInDegrees)
    {
        angleInDegrees = angleInDegrees % 360;
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Start Angle", startAngle);
        telemetry.update();
        float angleTurned;

        while (getLinearOpMode().opModeIsActive() && (angleTurned = findTurnCounterClockwise(startAngle)) < angleInDegrees)
        {
            setPower(-maxPower, -maxPower, maxPower, maxPower);
            telemetry.addData("Angle Turned", angleTurned);
            telemetry.update();
        }

        stop();
    }

    private LinearOpMode getLinearOpMode() {
        return (LinearOpMode) opMode;
    }

    private float findTurnClockwise(float startAngle)
    {
        // find current angular position
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        // convert angles to 0 - 360 scale
        startAngle = convertAngularScale(startAngle);
        currentAngle = convertAngularScale(currentAngle);

        float angleTurned;

        if (currentAngle >= startAngle)
        {
            angleTurned = currentAngle - startAngle;
        } else
        {
            angleTurned = 360 - (startAngle - currentAngle);
        }

        return angleTurned;
    }

    private float findTurnCounterClockwise(float startAngle)
    {
        // find current angular position
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // convert angles to 0 - 360 scale
        startAngle = convertAngularScale(startAngle);
        currentAngle = convertAngularScale(currentAngle);

        float angleTurned;

        if (currentAngle <= startAngle)
        {
            angleTurned = startAngle - currentAngle;
        } else
        {
            angleTurned = 360 - (currentAngle - startAngle);
        }

        return angleTurned;
    }

    private float convertAngularScale(float angle)
    {
        if (angle <= 0)
        {
            angle = Math.abs(angle);
        } else
        {
            angle = 180 + (180 - angle);
        }

        return angle;
    }

    public void strafeRight (double maxPower, double ditanceInInches){

    }

    public void strafeLeft(double maxPower, double distanceInInches) {

    }

    public void stop() {
        setPower(0, 0, 0, 0);
    }

    private void setPower(double drive, double rotate)
    {
        double frontLeftPower = drive + rotate;
        double backLeftPower = drive + rotate;
        double frontRightPower = drive - rotate;
        double backRightPower = drive - rotate;
        frontLeftPower = clip(frontLeftPower, -1, 1);
        backLeftPower = clip(backLeftPower, -1, 1);
        frontRightPower = clip(frontRightPower, -1, 1);
        backRightPower = clip(backRightPower, -1, 1);
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void setPower(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower)
    {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private double clip(double input, double min, double max)
    {
        if (input > max) return max;
        else if (input < min) return min;
        return input;
    }
}
