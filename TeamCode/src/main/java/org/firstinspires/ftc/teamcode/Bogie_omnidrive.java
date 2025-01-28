package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Bogie Fantastic Fight Mode", group="Robot")
public class Bogie_omnidrive  extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();

        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor tiltmotorA;
        private DcMotor tiltmotorB;
        private DcMotor slideMotor;
        // Servo objects
        private Servo elbowServo;
        private Servo wristServo;
        private Servo clawServo;


        private double clawOpen = 1;
        private double clawClose = 0;
        private int armTicks = 120;
        private double tiltPower = 1;
        private int slideTicks = 120;
        private double slidePower = .8;
        private int tiltStartPosition = 0;
        private int slideStartPosition = 0;
        private int currentIndex = 0; // 0 = elbow, 1 = wrist, 2 = claw
        private final double INCREMENT = 0.05; // Increment value
        final int SLIDE_FULL = 2300; // Full slide extension

        // Servo positions
        private double elbow = 0;  // Default position
        private double wrist = 0.65;  // Default position
        private double claw = clawClose;   // Default position
        // Position class to hold state values
        public static class Position {
            public double elbow;
            public double wrist;
            public int tilt;
            public int slide;

            public Position(double elbow, double wrist, int tilt, int slide) {
                this.elbow = elbow;
                this.wrist = wrist;
                this.tilt = tilt;
                this.slide = slide;
            }
        }

        // Enum for state machine states
        public enum RobotState {
            STEADY_STATE,
            PICKUP_POSITION,
            DRIVE_POSITION,
            SCORE_POSITION,
            START_POSITION
        }
        org.firstinspires.ftc.teamcode.Setup_Bot.RobotState currentState = org.firstinspires.ftc.teamcode.Setup_Bot.RobotState.STEADY_STATE;


        // Predefined positions
        private org.firstinspires.ftc.teamcode.Setup_Bot.Position pickupPosition;
        private org.firstinspires.ftc.teamcode.Setup_Bot.Position drivePosition;
        private org.firstinspires.ftc.teamcode.Setup_Bot.Position scorePosition;
        private org.firstinspires.ftc.teamcode.Setup_Bot.Position startPosition;

        @Override
        public void runOpMode() {
            tiltmotorA = hardwareMap.get(DcMotor.class, "tilt_motor"); // Exp Hub 1
            tiltmotorB = hardwareMap.get(DcMotor.class, "tilt_motor_2"); // Exp Hub 2
            slideMotor = hardwareMap.get(DcMotor.class, "slide_motor"); // Exp Hub 0
            // Map servos in the hardware map
            elbowServo = hardwareMap.get(Servo.class, "elbow");
            wristServo = hardwareMap.get(Servo.class, "wrist");
            clawServo = hardwareMap.get(Servo.class, "claw");

            // Initialize positions elbow wrist tilt slide
            pickupPosition = new org.firstinspires.ftc.teamcode.Setup_Bot.Position(0.75, 0.35, 500, 1275);
            drivePosition = new org.firstinspires.ftc.teamcode.Setup_Bot.Position(0.75, 0.35, 1500, 250);
            scorePosition = new org.firstinspires.ftc.teamcode.Setup_Bot.Position(1.0, 0.35, 5770, 2200);
            startPosition = new org.firstinspires.ftc.teamcode.Setup_Bot.Position(0, 0.65, 10, 10);
            // Wait for the game to start
            waitForStart();
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            tiltmotorA.setDirection(DcMotor.Direction.REVERSE);
            tiltmotorB.setDirection(DcMotor.Direction.REVERSE);
            tiltmotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tiltmotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltmotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltmotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); // Hub 0
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");  // Hub 2
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // Hub 1
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");  // Hub 3

            // ########################################################################################
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            while (opModeIsActive()) {

                compute_omni();

                // Switch between Elbow, Wrist, and Claw when 'Y' is pressed
                if (gamepad1.y) {
                    currentState = Setup_Bot.RobotState.START_POSITION;
                    currentIndex = (currentIndex + 1) % 3; // Cycle through 0, 1, 2
                    sleep(300); // Add debounce to avoid rapid cycli
                }
                //  state transitions based on gamepad input
                else if (gamepad1.a) {
                    currentState = org.firstinspires.ftc.teamcode.Setup_Bot.RobotState.PICKUP_POSITION;
                } else if (gamepad1.b) {
                    currentState = org.firstinspires.ftc.teamcode.Setup_Bot.RobotState.DRIVE_POSITION;
                    if (gamepad1.x) {
                        currentState = org.firstinspires.ftc.teamcode.Setup_Bot.RobotState.START_POSITION;
                    }
                } else if (gamepad1.x) {
                    currentState = org.firstinspires.ftc.teamcode.Setup_Bot.RobotState.SCORE_POSITION;
                }

                if (gamepad1.left_trigger > .2) {
                    clawServo.setPosition(clawClose);
                }
                else if (gamepad1.right_trigger > .2) {
                    clawServo.setPosition(clawOpen);
                }

                switch (currentState) {
                    case PICKUP_POSITION:
                        tiltmotorA.setTargetPosition(pickupPosition.tilt);
                        tiltmotorB.setTargetPosition(pickupPosition.tilt);
                        slideMotor.setTargetPosition(pickupPosition.slide);
                        elbowServo.setPosition(pickupPosition.elbow);
                        wristServo.setPosition(pickupPosition.wrist);
                        break;
                    case DRIVE_POSITION:
                        tiltmotorA.setTargetPosition(drivePosition.tilt);
                        tiltmotorB.setTargetPosition(drivePosition.tilt);
                        slideMotor.setTargetPosition(drivePosition.slide);
                        if (tiltmotorA.getCurrentPosition() > 1000){
                            elbowServo.setPosition(drivePosition.elbow);
                            wristServo.setPosition(drivePosition.wrist);
                        }

                        break;
                    case SCORE_POSITION:
                        updateArm(scorePosition);
                        break;

                    case START_POSITION:
                        if (tiltmotorA.getCurrentPosition() > 1000) {
                            clawServo.setPosition(clawClose);
                            wristServo.setPosition(startPosition.wrist);
                            sleep(1000); // wait for servos
                            updateArm(startPosition);
                        }

                    case STEADY_STATE:
                        positioner();
                        break;

                }
                updateTelemetry();
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

        private void positioner() {

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
                    else if (gamepad1.right_bumper) {
                        claw = Math.min(1, claw + INCREMENT);
                        sleep(200); // Add debounce
                    }
                    clawServo.setPosition(claw); // Update servo
                    break;
            }
        }
        private void updateArm(org.firstinspires.ftc.teamcode.Setup_Bot.Position position) {
            elbowServo.setPosition(position.elbow);
            wristServo.setPosition(position.wrist);
            tiltmotorA.setTargetPosition(position.tilt);
            tiltmotorB.setTargetPosition(position.tilt);
            slideMotor.setTargetPosition(position.slide);
        }
        private void updateTelemetry() {

            // Telemetry to display current values
            telemetry.addData("Current Selection", getCurrentSelection());
            telemetry.addData("Elbow Position", elbow);
            telemetry.addData("Wrist Position", wrist);
            telemetry.addData("Claw Position", claw);
            telemetry.addData("Tilt Position: ", tiltmotorA.getCurrentPosition()); // Get tilt and slide encoder values.
            telemetry.addData("Slide Position: ", slideMotor.getCurrentPosition()); // Added by Pinnacle.
            telemetry.addData("Current State", currentState);
            telemetry.update();
        }

        private void slide_tilt_manual_control() {

            if (gamepad1.dpad_down) { // 🔘 D-Pad Down
                int ticks = tiltmotorA.getCurrentPosition() - armTicks;
                tiltmotorA.setTargetPosition(ticks);
                tiltmotorB.setTargetPosition(ticks);
            }
            // going to control tilt
            if (gamepad1.dpad_up) { //
                int ticks = tiltmotorA.getCurrentPosition() + armTicks;
                tiltmotorA.setTargetPosition(ticks);
                tiltmotorB.setTargetPosition(ticks);
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

        }
        private void compute_omni() {


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

        }
    }


