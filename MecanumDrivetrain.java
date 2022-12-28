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

/**
 * This class implements a drive train powered by mecanum wheels.
 * <p>
 * Created by Rahul V, Arko, Anshul, Rahul D, Aarush on 8/19/2017.
 */


public class MecanumDrivetrain implements Drivetrain
{

    private static final double MIN_POWER = 0.1;
    private static final double PID_COEFF = 0.0002;
    private static final double TICKS_PER_ROTATION = 1120;
    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double WHEEL_CIRCUMFERENCE_IN_INCHES = WHEEL_DIAMETER_IN_INCHES * Math.PI;
    private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_IN_INCHES;
    private static final double STRAFE_POWER_FACTOR = 0.5;
    private static final double TICKS_PER_INCH_FOR_STRAFE = TICKS_PER_INCH * 1.33;

    private OpMode opMode;
    private Telemetry telemetry;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;
    
        /**
         * Constructor for autonomous
         *
         * @param opMode
         */
    public MecanumDrivetrain(OpMode opMode, boolean isAutonomous)
    {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        frontLeft = opMode.hardwareMap.dcMotor.get("FrontLeft");
        backLeft = opMode.hardwareMap.dcMotor.get("BackLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("FrontRight");
        backRight = opMode.hardwareMap.dcMotor.get("BackRight");

        telemetry.addData("Configuration", "Motors Found");
        telemetry.update();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

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

    /**
     * @param gamepad1_left_stick_x
     * @param gamepad1_left_stick_y
     * @param gamepad1_right_stick_x
     * @param gamepad1_right_stick_y
     */
    @Override
    public void drive(double gamepad1_left_stick_x, double gamepad1_left_stick_y, double gamepad1_right_stick_x, double gamepad1_right_stick_y)
    {
        setPower(gamepad1_right_stick_x * STRAFE_POWER_FACTOR, -gamepad1_left_stick_y, gamepad1_left_stick_x);
    }

    /**
     * @param maxPower
     * @param distanceInInches
     */
    @Override
    public void moveForward(double maxPower, double distanceInInches)
    {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition + distanceInInches * TICKS_PER_INCH;
        double distanceToTarget = targetPosition - currentPosition;
        double power = maxPower;
        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) < targetPosition)
        {
            distanceToTarget = targetPosition - currentPosition;
            //            if (distanceToTarget < TICKS_PER_ROTATION/2) {
            //                power = distanceToTarget * PID_COEFF;
            //                power = clip(power, MIN_POWER, maxPower);
            //            }
            setPower(power, power, power, power);
//            telemetry.addData("Current Position", currentPosition);
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Distance to Target", distanceToTarget);
//            telemetry.update();
        }


        stop();

//        telemetry.addData("Current Position", currentPosition);
//        telemetry.addData("Target Position", targetPosition);
//        telemetry.addData("Distance to Target", distanceToTarget);
//        telemetry.update();
    }

    /**
     * @param maxPower
     * @param distanceInInches
     */
    @Override
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

    /**
     * @param maxPower
     * @param distanceInInches
     */
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

    /**
     * @param maxPower
     * @param angleInDegrees
     */
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

    /**
     * @param maxPower
     * @param angleInDegrees
     */
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

    /**
     * @param maxPower
     * @param distanceInInches
     */
    @Override
    public void strafeRight(double maxPower, double distanceInInches)
    {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition + distanceInInches * TICKS_PER_INCH_FOR_STRAFE;
        double distanceToTarget = targetPosition - currentPosition;

        double power = maxPower;
        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) < targetPosition)
        {
            distanceToTarget = targetPosition - currentPosition;
            //            if (distanceToTarget < TICKS_PER_ROTATION/2) {
            //                power = distanceToTarget * PID_COEFF;
            //                power = clip(power, MIN_POWER, maxPower);
            //            }
            setPower(power, -power, -power, power);

            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.update();
        }


        stop();

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Distance to Target", distanceToTarget);
        telemetry.update();

    }

    /**
     * @param maxPower
     * @param distanceInInches
     */
    @Override
    public void strafeLeft(double maxPower, double distanceInInches)
    {
        double currentPosition = frontLeft.getCurrentPosition();

        telemetry.addData("Original Position", currentPosition);
        telemetry.update();

        double targetPosition = currentPosition - distanceInInches * TICKS_PER_INCH_FOR_STRAFE;
        double distanceToTarget = currentPosition - targetPosition;

        double power = maxPower;
        while (getLinearOpMode().opModeIsActive() && (currentPosition = frontLeft.getCurrentPosition()) > targetPosition)
        {
            distanceToTarget = currentPosition - targetPosition;
            //            if (distanceToTarget < TICKS_PER_ROTATION/2) {
            //                power = distanceToTarget * PID_COEFF;
            //                power = clip(power, MIN_POWER, maxPower);
            //            }
            setPower(-power, power, power, -power);

            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.update();
            getLinearOpMode().idle();
        }


        stop();

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Distance to Target", distanceToTarget);
        telemetry.update();
    }

    /**
     *
     */
    @Override
    public void stop()
    {
        setPower(0, 0, 0, 0);
    }

     /**
     * Determines the net power for the motors of the mecanum drive train during driver controlled play.
     *
     * @param strafe
     * @param drive
     * @param rotate
     */
    private void setPower(double strafe, double drive, double rotate)
    {
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;
        frontLeftPower = clip(frontLeftPower, -1, 1);
        backLeftPower = clip(backLeftPower, -1, 1);
        frontRightPower = clip(frontRightPower, -1, 1);
        backRightPower = clip(backRightPower, -1, 1);
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Sets the power to the motors directly.
     *
     * @param frontLeftPower
     * @param backLeftPower
     * @param frontRightPower
     * @param backRightPower
     */
    public void setPower(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower)
    {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Clips the motor power such that it is the min - max range.
     *
     * @param input
     * @param min
     * @param max
     * @return
     */
    private double clip(double input, double min, double max)
    {
        if (input > max) return max;
        else if (input < min) return min;
        return input;
    }

    /**
     * @return
     */
    private LinearOpMode getLinearOpMode()
    {
        return (LinearOpMode) opMode;
    }

    /**
     * Converts gyro reading from 0 -> -180 and 180 -> 0 scale to 0 -> 360 scale.
     *
     * @param angle
     * @return
     */
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

    /**
     * Calculates the robot turn in clockwise direction from the starting angular position.
     *
     * @param startAngle
     * @return
     */
    private float findTurnClockwise(float startAngle)
    {
        // find current angular position
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        telemetry.addData("Start Angle Before Conversion", startAngle);
//        telemetry.addData("Current Angle Before Conversion", currentAngle);

        // convert angles to 0 - 360 scale
        startAngle = convertAngularScale(startAngle);
        currentAngle = convertAngularScale(currentAngle);

//        telemetry.addData("Start Angle After Conversion", startAngle);
//        telemetry.addData("Current Angle After Conversion", currentAngle);
//        telemetry.update();

        // calculate the angle turned from the start angular start position.
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

    /**
     * Calculates the robot turn in counter clockwise direction from the starting angular position.
     *
     * @param startAngle
     * @return
     */
    private float findTurnCounterClockwise(float startAngle)
    {
        // find current angular position
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        telemetry.addData("Start Angle Before Conversion", startAngle);
//        telemetry.addData("Current Angle Before Conversion", currentAngle);
//        telemetry.update();

        // convert angles to 0 - 360 scale
        startAngle = convertAngularScale(startAngle);
        currentAngle = convertAngularScale(currentAngle);

//        telemetry.addData("Start Angle After Conversion", startAngle);
//        telemetry.addData("Current Angle After Conversion", currentAngle);
//        telemetry.update();

        // calculate the angle turned from the start angular position
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
    
    public float getRobotOrientation()
    {
        // find current angular position
        float currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Current orientation", currentOrientation);
        telemetry.update();
        return currentOrientation;
    }
    
    public float calculateRobotOrientationError(float originalOrientation)
    {
        return findTurnClockwise(originalOrientation);
    }
}