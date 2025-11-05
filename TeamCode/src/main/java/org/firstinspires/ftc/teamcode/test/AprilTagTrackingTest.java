/*
    Program to test autonomous AprilTag tracking with the Limelight 3A.
 */
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="AprilTag Tracking Test", group="Test")
@Disabled
public class AprilTagTrackingTest extends LinearOpMode {

    // Declare drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    // Limelight object (replace with your actual Limelight integration)
    // For example, you might have a custom Limelight class or use a library
    private Limelight3A limelight;

    // Target AprilTag ID
    private static final int TARGET_TAG_ID = 20; // Change to the ID of your target AprilTag

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Set motor directions for mecanum drive
        //frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Limelight (replace with your actual initialization)
        //limelight = new Limelight(hardwareMap, "Limelight 3A"); // Assuming a custom Limelight class
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(9);    // specifies which pipeline to use
        //limelight.setPollRateHz(100);   // set how often we ask Limelight for data (100 times per second)
        limelight.start();  // start polling Limelight for data.

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double KpTurn = 0.05;   // proportional control constant for turn
            double KpForward = 0.02;  // proportional control constant for forward

            // Get Limelight results (replace with your actual Limelight API calls)
            LLResult result = limelight.getLatestResult();

            // FORWARD/BACKWARD or LONGITUDINAL = left joystick y axis (robot centric)
            double forward = 0;
            double strafe = 0;
            double turn = 0;

            // Detect and use AprilTag data for autonomous tracking using LLResult
            if (result != null && result.isValid()) {
                // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
                // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
                double tx = result.getTx(); // how far left or right the target is (degrees)
                double ty = result.getTy(); // how far up or down the target is (degrees)
                double ta = result.getTa(); // how big the target looks (0%-100% of the image)

                // Implement PID or other control logic to center on the tag
                // For example, adjust 'turn' based on 'tx'
                turn = (tx * KpTurn);

                // You could also adjust 'forward' based on 'ty' or 'ta'
                //forward = (ty * KpForward); // move forward/backward to center vertically

                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", ty);
                telemetry.addData("Ta", ta);

                Pose3D botpose = result.getBotpose();   // MegaTag 1
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("BotPose", botpose.toString());
                    telemetry.addData("Location", "(" + x + ", " + y + ")");
                    telemetry.addData("Yaw", botpose.getOrientation().getYaw());
                }
            } else {
                telemetry.addData("Status", "No AprilTag detected.");
            }

/*
            // Detect and use AprilTag data for autonomous tracking using FiducialResults
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    for (LLResultTypes.FiducialResult target : fiducialResults) {
                        if (target.getFiducialId() == TARGET_TAG_ID) {
                            // Found the desired AprilTag
                            double tx = target.getTargetXDegrees(); // how far left or right the target is (degrees)
                            double ty = target.getTargetYDegrees(); // how far up or down the target is (degrees)
                            double ta = target.getTargetArea();     // how big the target looks (0%-100% of the image)

                            // Implement PID or other control logic to center on the tag
                            // For example, adjust 'turn' based on 'tx'
                            turn = tx * 0.05; // simple proportional control

                            // You could also adjust 'forward' based on 'ty' or 'ta'
                            // forward = -(ty * 0.02); // move forward/backward to center vertically

                            telemetry.addData("Tracking AprilTag", TARGET_TAG_ID);
                            telemetry.addData("Tx", tx);
                            telemetry.addData("Ty", ty);
                            telemetry.addData("Ta", ta);
                            break; // Exit loop after finding the desired tag
                        }
                    }
                } else {
                    telemetry.addData("Status", "Target AprilTag ID not detected.");
                }
            } else {
                telemetry.addData("Status", "No AprilTag detected.");
            }
 */

            // Move robot so that it's centered on target AprilTag
            moveRobot(forward, strafe, turn, 0.6); // speedLimiter reduces movement speed to a specified % of maximum (1.0).  USed for outreach events.

            telemetry.update();
        }
        limelight.stop();   // Stop Limelight data polling.
    }

    public void moveRobot(double y, double x, double rx, double speedLimiter) {
        // Function to move robot.
        // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.

        // Calculate the power needed for each wheel based on the amount of forward (y), strafe (x), and turn (rx)
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;
        double backLeftPower = y - x + rx;

        // Normalize motor power to prevent exceeding 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // We multiply by speedLimiter so that power can be set lower for outreach events.
        frontLeftDrive.setPower(frontLeftPower * speedLimiter);
        frontRightDrive.setPower(frontRightPower * speedLimiter);
        backLeftDrive.setPower(backLeftPower * speedLimiter);
        backRightDrive.setPower(backRightPower * speedLimiter);
    }
}
