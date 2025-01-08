

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Setting instance variable for your arm motors: tilt and slide. Added by Pinnacle.

    private DcMotor tiltmotorA;
    private DcMotor tiltmotorB;
    private DcMotor slideMotor;
    public CRServo intake_motor = null; //the active intake servo
    public Servo wrist_motor = null; //the wrist servo


    // Values for arm power and encoder motion. Added by Pinnacle.
    private int tiltStartPosition = 0;
    private int slideStartPosition = 0;
    private int armTicks = 120;
    private double tiltPower = 1;
    private int slideTicks = 120;
    private double slidePower = .8;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 0.1667;
    final double WRIST_TUCK = 0.1;  // servor ~= 0
    final double WRIST_SIDE = 0.5;  // servo 90
    final double WRIST_OUT = 0.85;  // servo ~= 180
    final double ACTIVE_TRIGGER = 0.2;  // Arbitrary bigger than 0

    final int SLIDE_FULL = 2000; // Full slide extension


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

        tiltmotorA = hardwareMap.get(DcMotor.class, "tilt_motor"); // Exp Hub 1
        tiltmotorB = hardwareMap.get(DcMotor.class, "tilt_motor_2"); // Exp Hub 2
        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor"); // Exp Hub 0
        intake_motor = hardwareMap.get(CRServo.class, "intake_motor");// Servo 0
        wrist_motor = hardwareMap.get(Servo.class, "wrist_motor");// Servo 1

        // ########################################################################################
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
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        tiltmotorA.setDirection(DcMotor.Direction.REVERSE);
        tiltmotorB.setDirection(DcMotor.Direction.REVERSE);
        tiltmotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltmotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tiltmotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tiltmotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(slideStartPosition);
        tiltmotorA.setTargetPosition(tiltStartPosition);
        tiltmotorB.setTargetPosition(tiltStartPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltmotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltmotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // intake_motor = hardwareMap.get(CRServo.class, "intake");
        //wrist_motor = hardwareMap.get(Servo.class, "wrist");
        intake_motor.setPower(INTAKE_OFF);
        wrist_motor.setPosition(WRIST_TUCK);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Tilt Position: ", tiltmotorA.getCurrentPosition()); // Get tilt and slide encoder values.
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
            double yaw = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial + lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial - lateral - yaw;


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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Sets power to the arm motors. Added by Pinnacle.
            tiltmotorA.setPower(tiltPower);
            tiltmotorB.setPower(tiltPower);
            slideMotor.setPower(slidePower);
//Sets power to hand motors. Added by 11706
            intake_motor.setPower(intake_motor.getPower());
            wrist_motor.setPosition(wrist_motor.getPosition());
            // Controls for your tilt motor. Added by Pinnacle.
            if (gamepad1.dpad_down) { // 🔘 D-Pad Down
                //tiltmotorA.setTargetPosition(tiltmotorA.getCurrentPosition() + armTicks);
                // if (tiltmotorA.getCurrentPosition() < 5870) {
//
               //{
                    int ticks = tiltmotorA.getCurrentPosition() - armTicks;
                    tiltmotorA.setTargetPosition(ticks);
                    tiltmotorB.setTargetPosition(ticks);
             //   }
                //   }
//                else [
//                    tiltmotorA.setTargetPosition(5870);
//                ]
            }
                // going to control tilt
                if (gamepad1.dpad_up) { // 🔘 D-Pad Up
                    //tiltmotorA.setTargetPosition(tiltmotorA.getCurrentPosition() - armTicks);
                    // if (tiltmotorA.getCurrentPosition() > 1740) {
//                    if (tiltmotorA.getCurrentPosition() - armTicks < 1740) {
//                        tiltmotorA.setTargetPosition(1740);
//                        tiltmotorB.setTargetPosition(1740);
//                    } else
//                  {
                        int ticks = tiltmotorA.getCurrentPosition() + armTicks;
                        tiltmotorA.setTargetPosition(ticks);
                        tiltmotorB.setTargetPosition(ticks);
                 //   }
                    //  }

                }

                // Modified by Jayla, extend the slide
                if (gamepad1.dpad_right) { // 🔘 D-Pad Right
                    int current_slide_location = slideMotor.getCurrentPosition();
                    if (current_slide_location < SLIDE_FULL) {
                        slideMotor.setTargetPosition(current_slide_location + slideTicks);
                    } else {
                        slideMotor.setTargetPosition(SLIDE_FULL);
                    }

                }
// retracting the slide with left gamepad
                if (gamepad1.dpad_left) { // 🔘 D-Pad Left
                    int current_slide_location = slideMotor.getCurrentPosition();
                    if (current_slide_location  - slideTicks > 0) {
                        slideMotor.setTargetPosition(current_slide_location - slideTicks);
                    } else {
                        slideMotor.setTargetPosition(0);
                    }
                }

                if (gamepad1.a) {
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
                    int ticks = tiltmotorA.getCurrentPosition();

                    tiltmotorA.setTargetPosition(ticks);
                    tiltmotorB.setTargetPosition(ticks);
                }
                if (gamepad1.left_bumper) {
                    intake_motor.setPower(INTAKE_COLLECT);
                } else if (gamepad1.right_bumper) {
                    intake_motor.setPower(INTAKE_DEPOSIT);
                }
                else{
                    intake_motor.setPower(INTAKE_OFF);
                }

                if (gamepad1.left_trigger > ACTIVE_TRIGGER) {
                    wrist_motor.setPosition(WRIST_OUT);
                } else {
                    if (gamepad1.right_trigger > ACTIVE_TRIGGER) {
                        wrist_motor.setPosition(WRIST_SIDE);
                    } else {
                        if (gamepad1.b) {
                            wrist_motor.setPosition(WRIST_TUCK);
                        }
                    }
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Tilt Position: ", tiltmotorA.getCurrentPosition()); // Get tilt and slide encoder values.
                telemetry.addData("Slide Position: ", slideMotor.getCurrentPosition()); // Added by Pinnacle.
                telemetry.update();
            }
        }
    }





