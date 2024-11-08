/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Setting instance variable for your arm motors: tilt and slide. Added by Pinnacle.
    private DcMotor tiltMotor;
    private DcMotor slideMotor;
    public CRServo intake_motor = null; //the active intake servo
    public Servo wrist_motor = null; //the wrist servo


    // Values for arm power and encoder motion. Added by Pinnacle.
    private int tiltStartPosition = 0;
    private int slideStartPosition = 0;
    private int armTicks = 120;
    private double tiltPower = 1;
    private int slideTicks = 120;
    private double slidePower = 1;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 0.1667;
    final double WRIST_FOLDED_OUT = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); // Hub 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");  // Hub 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // Hub 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");  // Hub 3

        /* As of writing this code, the motors are plugged in and configured. */
        /* Feel free to change as needed to match your desired config. - Added by Pinnacle */
        tiltMotor = hardwareMap.get(DcMotor.class, "tilt_motor"); // Exp Hub 1
        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor"); // Exp Hub 0
        intake_motor = hardwareMap.get(CRServo.class, "intake_motor");// Servo 0
        wrist_motor = hardwareMap.get(Servo.class, "wrist_motor");// Servo 1

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Sets the zero power behavior to resist gravity and inertia.
        // Sets the motors to run using encoder values.
        // Sets the motors to actively pursue positions defined by encoder ticks.
        // Sets starting target position to avoid initialization errors.
        // Added by Pinnacle.
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(slideStartPosition);
        tiltMotor.setTargetPosition(tiltStartPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_motor = hardwareMap.get(CRServo.class, "intake");
        wrist_motor = hardwareMap.get(Servo.class, "wrist");
        intake_motor.setPower(INTAKE_OFF);
        wrist_motor.setPosition(WRIST_FOLDED_OUT);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Tilt Position: ", tiltMotor.getCurrentPosition()); // Get tilt and slide encoder values.
        telemetry.addData("Slide Position: ", slideMotor.getCurrentPosition()); // Added by Pinnacle.
        telemetry.addData("Intake Position", intake_motor.getPower());
        telemetry.addData("Wrist Position", wrist_motor.getPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Sets power to the arm motors. Added by Pinnacle.
            tiltMotor.setPower(tiltPower);
            slideMotor.setPower(slidePower);
//Sets power to hand motors. Added by 11706
            intake_motor.setPower(intake_motor.getPower());
            wrist_motor.setPosition(wrist_motor.getPosition());
            // Controls for your tilt motor. Added by Pinnacle.
            if (gamepad1.dpad_down) { // 🔘 D-Pad Down
                tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + armTicks);
                if (tiltMotor.getCurrentPosition() < -5870) {
                    if (tiltMotor.getCurrentPosition() + armTicks > -5870) {
                        tiltMotor.setTargetPosition(-5870);
                    } else {
                        tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + armTicks);
                    }
                }
            }
            if (gamepad1.dpad_up) { // 🔘 D-Pad Up
                tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() - armTicks);
                if (tiltMotor.getCurrentPosition() < -1740) {
                    if (tiltMotor.getCurrentPosition() + armTicks > -1740) {
                        tiltMotor.setTargetPosition(-1740);
                    } else {
                        tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + armTicks);
                    }
                }
            }

            // Controls for your slide motor. Added by Pinnacle.
            if (gamepad1.dpad_right) { // 🔘 D-Pad Right
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + slideTicks);
                if (tiltMotor.getCurrentPosition() < -1303) {
                    if (tiltMotor.getCurrentPosition() + slideTicks > -1303) {
                        tiltMotor.setTargetPosition(-1303);
                    } else {
                        tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + slideTicks);
                    }
                }
            }

            if (gamepad1.dpad_left) { // 🔘 D-Pad Left
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - slideTicks);
                if (tiltMotor.getCurrentPosition() < 0) {
                    if (tiltMotor.getCurrentPosition() + slideTicks > 0) {
                        tiltMotor.setTargetPosition(0);
                    } else {
                        tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + slideTicks);
                    }

                    if (gamepad1.a) {
                        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
                        tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition());
                    }
                    if (gamepad1.left_bumper) {
                        double intakeCollect = INTAKE_COLLECT;
                    }
                    if (gamepad1.right_bumper) {
                        intake_motor.setPower(INTAKE_OFF);
                    }
                    if (gamepad1.y) {
                       intake_motor.setPower(INTAKE_DEPOSIT);
                        if (gamepad1.left_trigger > 0) {
                            wrist_motor.setPosition(0);
                            if (gamepad1.right_trigger > 0) {
                                wrist_motor.setPosition(1);
                            }
                            if (gamepad1.b) {
                              //  wrist.setPosition(0.5);
                            }
                        }

                        // Show the elapsed game time and wheel power.
                        telemetry.addData("Status", "Run Time: " + runtime.toString());
                        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                        telemetry.addData("Tilt Position: ", tiltMotor.getCurrentPosition()); // Get tilt and slide encoder values.
                        telemetry.addData("Slide Position: ", slideMotor.getCurrentPosition()); // Added by Pinnacle.
                        telemetry.update();
                    }
                }
            }
        }
    }
}






// max slide length = -1303
// min slide limit = 0
// max tilt limit = -5870
// min tilt limit = -1740