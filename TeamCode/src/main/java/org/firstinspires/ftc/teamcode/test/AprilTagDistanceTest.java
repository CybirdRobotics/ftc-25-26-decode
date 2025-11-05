package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="AprilTag Distance Test", group="Test")
@Disabled
public class AprilTagDistanceTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    public enum AllianceColor {
        RED,
        BLUE
    }
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;

    public boolean ATagVisible = false;

    @Override
    public void init() {

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
    }

    @Override
    public void start() {
        super.start();
        limelight.start();  // start polling Limelight for data.
    }

    @Override
    public void loop() {
        double distance = 0, bearing = 0;

        // Limelight MegaTag2 requires input (yaw) from the IMU for localization.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        /*
        // Get the current Limelight pipeline results
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
            // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
            double tx = result.getTx(); // how far left or right the target is (degrees)
            double ty = result.getTy(); // how far up or down the target is (degrees)
            double ta = result.getTa(); // how big the target looks relative to the frame of view
            distance = getDistance(ty); // calculate the estimated distance to target
            //bearing = -tx;    // bearing to target (botpose in field space, hence the negative sign)

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
        */

        // code segment from FTC 23511 (https://github.com/FTC-23511/Decode-2026/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commandbase/subsystems/Turret.java)
        double[] targetDegrees = getLimeLightTargetDegrees();

        if (targetDegrees != null) {
            double tx = targetDegrees[0];
            double ty = targetDegrees[1];
            ATagVisible = true;
            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);
        } else {
            ATagVisible = false;
        }
        telemetry.addData("AprilTag Visible", ATagVisible);


        telemetry.update();
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

    public LLStatus getLimelightStatus() {
        return limelight.getStatus();
    }

    public double[] getLimeLightTargetDegrees() {
        double[] targetDegrees = new double[2];
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                if ((ALLIANCE_COLOR.equals(AllianceColor.BLUE) && id == 20)
                        || (ALLIANCE_COLOR.equals(AllianceColor.RED) && id == 24)) {

                    targetDegrees[0] = fiducial.getTargetXDegrees();
                    targetDegrees[1] = fiducial.getTargetYDegrees();

                    return targetDegrees;
                }
            }
        }

        return null;
    }

    @Override
    public void stop() {
        super.stop();
        limelight.stop();
    }
}