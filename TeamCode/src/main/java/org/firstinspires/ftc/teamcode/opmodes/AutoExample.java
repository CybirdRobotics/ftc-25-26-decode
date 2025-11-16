package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name="Auto Example", group = "OpMode")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class AutoExample extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object (class)
    //Launcher launcher = new Launcher(); // create instance of Launcher object (class)
    private ElapsedTime driveTime = new ElapsedTime();    // create drive timer instance

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);    // initialize the drive subsystem.
        //launcher.init(hardwareMap);   // initialize the launcher subsystem.

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot initialized.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
        // Step 1: Wait 5 seconds before moving forward
        sleep(5000);

        // Step 2: Drive forward for 3 seconds
        drive.driveRobotRelative(0.5, 0, 0);
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < 1.4) {
            telemetry.addData("Path", "Leg 1: %4.1f seconds elapsed.", driveTime.seconds());
            telemetry.update();
        }

        // Step 3: Stop robot movement
        drive.stopRobot();

        telemetry.addLine("AUTO complete.");
        telemetry.update();
        sleep(1000);
    }
}