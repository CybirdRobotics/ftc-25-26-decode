// From Coach Pratt "GoBILDAs DECODE Starter Robot is Good.  Let's Make it Great" YouTube video on improving programming techniques.
// https://youtu.be/zfPU8Vy9yiY?si=A0EJxVQ1khANI9vK

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Mecanum Drive (Coach Pratt)", group = "OpMode")
@Disabled
public class TeleOpExample extends OpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object (class)
    //Launcher launcher = new Launcher(); // create instance of Launcher object (class)

    @Override
    public void init() {
        drive.init(hardwareMap);
        //launcher.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {  // press the left bumper for robot centric drive
            drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {    // field centric drive
            drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        /*
         * Here we give the user control to start/stop the launcher motor without automatically
         * queuing a shot.

        if (gamepad1.y) {           // use gamepad Y to start flywheel launcher
            launcher.startLauncher();
        } else if (gamepad1.b) {    // use gamepad B to stop flywheel launcher
            launcher.stopLauncher();
        }

        // state machine
        launcher.state();

        telemetry.addData("State", launcher.getState());
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
         */
    }
}
