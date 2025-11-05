/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Mecanum Drive (Field Relative)", group = "OpMode")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOpFieldRelative extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings specified here as parameters must match the names assigned in the robot controller configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // TODO: Make sure all motors are facing the correct direction.
        //frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoder for constant power resulting in increased accuracy. Comment out if motor encoders not connected.
        //frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight camera.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(9);    // specifies which pipeline to use, in this case pipeline 9 is configured for AprilTag 20.
        //limelight.setPollRateHz(100);   // set how often we ask Limelight for data (100 times per second)

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();

        waitForStart();

        limelight.start();  // start polling Limelight for data.

        // Run until the driver presses stop
        while (opModeIsActive()) {
            double KpTurn = 0.05;     // proportional control constant for turn
            double KpDistance = 0.1;  // proportional control constant for distance
            double distanceError = 0, headingError = 0;
            double distance = 0;
            double targetDistance = 24; // desired distance to target in inches

            telemetry.addLine("Press A to reset Yaw.");
            telemetry.addLine("Hold LEFT bumper to drive in robot relative mode.");
            telemetry.addLine("Hold RIGHT bumper for auto scoring alignment.");

            // When gamepad1.a is pressed, reset the Yaw to 0 based on the orientation relative to the robot's position
            if (gamepad1.a) {
                imu.resetYaw();
            }

            // Limelight MegaTag2 requires input (yaw) from the IMU for localization.
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            // Get the current Limelight pipeline results
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
                // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
                double tx = result.getTx(); // how far left or right the target is (degrees)
                double ty = result.getTy(); // how far up or down the target is (degrees)
                double ta = result.getTa(); // how big the target looks relative to the frame of view
                distance = getDistance(ty); // calculate the estimated distance to target
                distanceError = (distance - targetDistance) * KpDistance;
                headingError = (tx * KpTurn);   // implement PID control logic to center on the AprilTag. For example, adjust 'turn' based on 'tx'

                // Send telemetry data to the Driver Hub
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("Distance to Target", distance);

                Pose3D botpose = result.getBotpose_MT2();   // get the 3D object position and orientation
                if (botpose != null) {
                    telemetry.addData("\nBotpose", botpose.toString());
                } else {
                    telemetry.addLine("No Botpose data available.");
                }
            } else {
                telemetry.addLine("No AprilTag detected.");
            }

            if (gamepad1.right_bumper) {    // align robot with target, then press right_bumper for auto shooting alignment
                driveRobotRelative(distanceError, 0, headingError);
            } else if (gamepad1.left_bumper) {  // press the left bumper for robot centric drive
                driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            } else {    // field centric drive
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            }
            telemetry.update();
        }

        if (isStopRequested()) {
            limelight.stop();   // stop Limelight data polling.
        }
    }

    private void driveFieldRelative(double forward, double strafe, double turn) {
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

        double speedLimiter = 0.4; // limits robot drive speed - used for outreach events (default = 1.0)

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

    public double getDistance(double a2) {
        // Calculate distance to target AprilTag.
        // From the Limelight documentation, https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance.
        // Solve for distance = (h2-h1) / tan(a1+a2) where the height of the target (h2) is known because it is a property of the field (e.g. AprilTag),
        // the height of your camera lens above the floor (h1) is known as is its mounting angle (a1).
        // The Limelight can tell you the Ty angle to the target (a2).  Note, The tan() usually expects an input measured in radians.
        // To convert an angle measurement from degrees to radians, multiply by (3.14159/180.0).

        // Degrees your limelight is pitched back (rotated) from vertical
        double limelightPitchAngle = 15;    // 15 degrees in LL Pitch config

        // Distance from the center of the Limelight lens to the floor in inches
        double limelightHeight = 2.5;   // 0.0635 meters in LL Up config

        // Distance from the center of the robot to the Limelight lens in inches
        double limelightForwardOffset = 6.375;  // 0.161925 meters in LL Forward config

        // Distance from the target to the floor in inches
        // Note, goal is 38.75 in (98.45 cm) tall and AprilTag center is located 9.25 in (23.5 cm) from top of goal (CM pgs. 69 & 77).
        double targetHeight = 29.5; // center of AprilTag on DECODE goal in inches (38.75 - 9.25)

        double angleToTargetDegrees = limelightPitchAngle + a2;
        double angleToTargetRadians = angleToTargetDegrees * (3.14159 / 180.0);

        // calculate distance in inches
        return (targetHeight - limelightHeight) / Math.tan(angleToTargetRadians);
    }
}