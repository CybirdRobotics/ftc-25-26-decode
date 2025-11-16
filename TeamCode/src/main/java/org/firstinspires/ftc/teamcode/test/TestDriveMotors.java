/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station. When a selection is made from
 * the menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Test Drive Motors", group = "Test")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class TestDriveMotors extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips.
        // TODO: Make sure all motors are facing the correct direction.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset the motor encoder so that it reads zero ticks
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        // Note, RUN_WITHOUT_ENCODER does not disable the encoder; instead tells the SDK not to use the motor encoder
        // for built-in velocity control.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting.
        telemetry.addLine("Robot ready.");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // Get number of encoder ticks per rotation for the motor type specified in the Control Hub configuration
        double ticksPerRotation = frontLeftDrive.getMotorType().getTicksPerRev();

        // Test motor directions. Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot by adjusting your Robot Controller Configuration, if necessary.
        //   2) Then make sure they run in the correct direction by modifying the the setDirection() calls above.
        double frontLeftPower  = gamepad1.x ? 0.4 : 0.0;  // X or PS4: □ (square)
        double rearLeftPower   = gamepad1.a ? 0.4 : 0.0;  // A or PS4: X (cross)
        double frontRightPower = gamepad1.y ? 0.4 : 0.0;  // Y or PS4: ∆ (triangle)
        double rearRightPower  = gamepad1.b ? 0.4 : 0.0;  // B or PS4: O (circle)

        // Write effectors
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(rearLeftPower);
        backRightDrive.setPower(rearRightPower);

        // Update telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Encoder ticks per rotation: ", ticksPerRotation);
        telemetry.addData("Number of rotations: ", frontLeftDrive.getCurrentPosition() / ticksPerRotation);
        
        telemetry.addLine()
                .addData("FrontLeft Power", "%4.2f", frontLeftPower)
                .addData("FrontLeft Encoder ", "%s", frontLeftDrive.getCurrentPosition());
        telemetry.addLine()
                .addData("RearLeft Power", "%4.2f", rearLeftPower)
                .addData("RearLeft Encoder ", "%s", backLeftDrive.getCurrentPosition());
        telemetry.addLine()
                .addData("FrontRight Power", "%4.2f", frontRightPower)
                .addData("FrontRight Encoder ", "%s", frontRightDrive.getCurrentPosition());
        telemetry.addLine()
                .addData("RearRight Power", "%4.2f", rearRightPower)
                .addData("RearRight Encoder ", "%s", backRightDrive.getCurrentPosition());
        
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}