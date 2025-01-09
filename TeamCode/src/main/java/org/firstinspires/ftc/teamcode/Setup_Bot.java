package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Adjuster with Telemetry", group = "TeleOp")
public class Setup_Bot extends LinearOpMode {


    private DcMotor tiltmotorA;
    private DcMotor tiltmotorB;
    private DcMotor slideMotor;
    // Servo objects
    private Servo elbowServo;
    private Servo wristServo;
    private Servo clawServo;

    // Servo positions
    private double elbow = 0.5;  // Default position
    private double wrist = 0.5;  // Default position
    private double claw = 0.5;   // Default position

    private int armTicks = 120;
    private double tiltPower = 1;
    private int slideTicks = 120;
    private double slidePower = .8;
    private int tiltStartPosition = 0;
    private int slideStartPosition = 0;
    private int currentIndex = 0; // 0 = elbow, 1 = wrist, 2 = claw
    private final double INCREMENT = 0.05; // Increment value
    final int SLIDE_FULL = 2000; // Full slide extension

    @Override
    public void runOpMode() {

        tiltmotorA = hardwareMap.get(DcMotor.class, "tilt_motor"); // Exp Hub 1
        tiltmotorB = hardwareMap.get(DcMotor.class, "tilt_motor_2"); // Exp Hub 2
        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor"); // Exp Hub 0
        // Map servos in the hardware map
        elbowServo = hardwareMap.get(Servo.class, "elbow");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");

        // Wait for the game to start
        waitForStart();
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
        // Sets power to the arm motors. Added by Pinnacle.
        tiltmotorA.setPower(tiltPower);
        tiltmotorB.setPower(tiltPower);
        slideMotor.setPower(slidePower);

        while (opModeIsActive()) {
            //Sets power to hand motors. Added by 11706
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


            // Switch between Elbow, Wrist, and Claw when 'Y' is pressed
            if (gamepad1.y) {
                currentIndex = (currentIndex + 1) % 3; // Cycle through 0, 1, 2
                sleep(300); // Add debounce to avoid rapid cycling
            }

            // Act on the currently selected double
            switch (currentIndex) {
                case 0: // Elbow
                    if (gamepad1.left_bumper) {
                        elbow = Math.max(0, elbow - INCREMENT);
                        sleep(200); // Add debounce
                    }
                    if (gamepad1.right_bumper) {
                        elbow = Math.min(1, elbow + INCREMENT);
                        sleep(200); // Add debounce
                    }
                    elbowServo.setPosition(elbow); // Update servo
                    break;
                case 1: // Wrist
                    if (gamepad1.left_bumper) {
                        wrist = Math.max(0, wrist - INCREMENT);
                        sleep(200); // Add debounce
                    }
                    if (gamepad1.right_bumper) {
                        wrist = Math.min(1, wrist + INCREMENT);
                        sleep(200); // Add debounce
                    }
                    wristServo.setPosition(wrist); // Update servo
                    break;
                case 2: // Claw
                    if (gamepad1.left_bumper) {
                        claw = Math.max(0, claw - INCREMENT);
                        sleep(200); // Add debounce
                    }
                    if (gamepad1.right_bumper) {
                        claw = Math.min(1, claw + INCREMENT);
                        sleep(200); // Add debounce
                    }
                    clawServo.setPosition(claw); // Update servo
                    break;
            }

            // Telemetry to display current values
            telemetry.addData("Current Selection", getCurrentSelection());
            telemetry.addData("Elbow Position", elbow);
            telemetry.addData("Wrist Position", wrist);
            telemetry.addData("Claw Position", claw);
            telemetry.addData("Tilt Position: ", tiltmotorA.getCurrentPosition()); // Get tilt and slide encoder values.
            telemetry.addData("Slide Position: ", slideMotor.getCurrentPosition()); // Added by Pinnacle.
            telemetry.update();
        }
    }

    // Helper method to get the name of the current servo being adjusted
    private String getCurrentSelection() {
        switch (currentIndex) {
            case 0: return "Elbow";
            case 1: return "Wrist";
            case 2: return "Claw";
            default: return "Unknown";
        }
    }
}
