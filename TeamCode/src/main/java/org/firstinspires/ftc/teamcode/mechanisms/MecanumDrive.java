package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private IMU imu;

    private double speedLimiter = 0.5;  // speedLimiter reduces movement speed to a specified % of maximum (1.0). Used for outreach events.

    public void init(HardwareMap hwMap) {

        // Initialize the hardware variables. Note that the strings specified here as parameters must match the names assigned in the robot controller configuration.
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // TODO: Make sure all motors are facing the correct direction.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Use encoder for constant power resulting in increased accuracy. Comment out if motor encoders not connected.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void resetYaw() {
        imu.resetYaw();
    }
    public void driveFieldRelative(double forward, double strafe, double turn) {
        // Drive robot from the field frame of reference (field relative).

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, turn angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and strafe amounts
        driveRobotRelative(newForward, newStrafe, turn);
    }

    public void driveRobotRelative(double y, double x, double rx) {
        // Drive robot from the robot frame of reference (robot relative).
        // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.

        // Normalize motor power. This ensures all the powers are scaled proportionally and remain in the range of [-1.0, 1.0].
        double scaleFactor = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        // Calculate the power needed for each wheel based on the amount of forward (y), strafe (x), and turn (rx)
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double frontLeftPower = ((y + x + rx) / scaleFactor) * speedLimiter;
        double backLeftPower = ((y - x + rx) / scaleFactor) * speedLimiter;
        double frontRightPower = ((y - x - rx) / scaleFactor) * speedLimiter;
        double backRightPower = ((y + x - rx) / scaleFactor) * speedLimiter;

        // Set scaled motor powers (with limiter).
        frontLeftDrive.setPower(squarePower(frontLeftPower));
        backLeftDrive.setPower(squarePower(backLeftPower));
        frontRightDrive.setPower(squarePower(frontRightPower));
        backRightDrive.setPower(squarePower(backRightPower));
    }

    private static double squarePower(double power) {
        // Function to square motor power to allow for better micro control.  Returns a double.
        return power * Math.abs(power);  // square magnitude of input while maintaining the sign
    }

    public void stopRobot() {
        // Stop robot movement.
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}


