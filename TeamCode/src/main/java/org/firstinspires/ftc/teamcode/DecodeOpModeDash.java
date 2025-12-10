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

package org.firstinspires.ftc.teamcode;

// import com.acmerobotics.dashboard.FtcDashboard;
// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

import java.util.List;
import java.util.Objects;

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

// @Config
@TeleOp(name="DecodeOpModeWithDash", group="Linear OpMode")
//@Disabled
public class DecodeOpModeDash extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx flywheel = null;
    private DcMotor coreHex = null;
    private DcMotorEx intake = null;
    private Servo lifter = null;
    private CRServo servo = null;




    private Limelight3A limelight = null;
    // Setting our velocity targets. These values are in ticks per second!
    public static int bankVelocity = 1350;
    public static int firstBankVelocity = 1350;
    public static int farVelocity = 1625;
    public static final int maxVelocity = 1625;
    private static final String TELEOP = "TELEOP";
    private static final String AUTO_BLUE_NEAR = "AUTO BLUE NEAR";
    private static final String AUTO_BLUE_FAR = "AUTO BLUE FAR";
    private static final String AUTO_RED_NEAR = " AUTO RED NEAR";
    private static final String AUTO_RED_FAR = " AUTO RED FAR";

    private static final String RED = "RED";
    private static final String BLUE = "BLUE";

    private String operationSelected = TELEOP;

    private String allianceSelected = RED;


    private ElapsedTime autoLaunchTimer = new ElapsedTime();
    private ElapsedTime autoDriveTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();
    private double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);

    public static double pidP = 45.0;
    public static double pidF = 15.0;
    public static double pidI = 0.0;
    public static double pidD = 0.0;

    private boolean reduction_active = false;
private boolean intake_active = false;
private boolean servo_active_for_intake = false;
    private boolean autoAimEnabled = false;
    private double aimFrontLeft = 0.0;
    private double aimFrontRight = 0.0;
    private double aimBackLeft = 0.0;
    private double aimBackRight = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        servo = hardwareMap.get(CRServo.class, "servo");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lifter = hardwareMap.get(Servo.class, "lifter");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

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
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //intake.setDirection(DcMotor.Direction.REVERSE);
        //lifter.setDirection(Servo.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(pidP, pidI, pidD, pidF);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        coreHex.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setPower(0);

        limelight.pipelineSwitch(0);
        limelight.start();

        //On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
        while (opModeInInit()) {
            operationSelected = selectOperation(operationSelected, gamepad1.psWasPressed());
            allianceSelected = selectAlliance(allianceSelected, gamepad1.xWasPressed());
            telemetry.setMsTransmissionInterval(100);

            telemetry.update();
        }
        waitForStart();
        if (operationSelected.equals(AUTO_BLUE_FAR)) {
            doAutoBlueFar();
        } else if (operationSelected.equals(AUTO_BLUE_NEAR)) {
            doAutoBlueNear();
        } else if (operationSelected.equals(AUTO_RED_FAR)) {
            doAutoRedFar();
        } else if (operationSelected.equals(AUTO_RED_NEAR)) {
            doAutoRedNear();
        } else {
            autoAimEnabled = true;
            doTeleOp();
        }
    }

    private void doTeleOp() {
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // FtcDashboard dashboard = FtcDashboard.getInstance();
            // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            if(gamepad1.circleWasPressed()) {
                intake_active = !intake_active;
            }

            if(intake_active) {
                ((DcMotorEx) intake).setVelocity(6000);
                lifter.setPosition(1.0);
                servo_active_for_intake = true;
            } else {
                ((DcMotorEx) intake).setVelocity(0);
                lifter.setPosition(0.25);
                servo_active_for_intake = false;
            }

            double max;

            if(gamepad1.psWasPressed()) {
                reduction_active = !reduction_active;
            }

            double reduction = 1.0;
            if(reduction_active) {
                reduction = 0.4;
            }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = reduction * -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = reduction * gamepad1.left_stick_x;
            double yaw = reduction * gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }
            boolean autoAimActive = setFlywheelVelocity();

            // Send calculated power to wheels
            if(!autoAimActive) {
                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);
            }

            manualCoreHexAndServoControl();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
            telemetry.addData("Flywheel Power", flywheel.getPower());
            telemetry.addData("Reduction Active", reduction_active);
            telemetry.update();
        }
    }

    /**
     * If the PS/Home button is pressed, the robot will cycle through the OpMode options following the if/else statement here.
     * The telemetry readout to the Driver Station App will update to reflect which is currently selected for when "play" is pressed.
     */
    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP)) {
                state = AUTO_BLUE_FAR;
            } else if (state.equals(AUTO_BLUE_FAR)) {
                state = AUTO_BLUE_NEAR;
            } else if (state.equals(AUTO_BLUE_NEAR)) {
                state = AUTO_RED_FAR;
            } else if (state.equals(AUTO_RED_FAR)) {
                state = AUTO_RED_NEAR;
            } else if (state.equals(AUTO_RED_NEAR)) {
                state = TELEOP;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
            }
        }
        telemetry.addLine("Press Home Button to cycle options");
        telemetry.addData("CURRENT SELECTION", state);
        if (state.equals(AUTO_BLUE_NEAR) || state.equals(AUTO_RED_NEAR) || state.equals(AUTO_RED_FAR) || state.equals(AUTO_BLUE_FAR)) {
            telemetry.addLine("Please remember to enable the AUTO timer!");
        }
        telemetry.addLine("Press START to start your program");
        return state;
    }

    private String selectAlliance(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(RED)) {
                state = BLUE;
            } else if (state.equals(BLUE)) {
                state = RED;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
            }
        }
        telemetry.addLine("Press X Button to cycle alliance");
        telemetry.addData("CURRENT ALLIANCE", state);
       return state;
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
 private void manualCoreHexAndServoControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            coreHex.setPower(0.5);
        } else if (gamepad1.triangle) {
            coreHex.setPower(-0.5);
        }
        // Manual control for the hopper's servo
        if (servo_active_for_intake) {
            servo.setPower(1);
        } else if (gamepad1.dpad_left) {
            servo.setPower(-1);
        } else if (gamepad1.dpad_right) {
            servo.setPower(1);
        }
    }

    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private boolean setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            return farPowerAuto();
        } else if (gamepad1.right_bumper) {
            bankShotAuto();
        } else if (gamepad1.square) {
            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        } else {
            ((DcMotorEx) flywheel).setVelocity(0);
            coreHex.setPower(0);
            // The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!
            if (!gamepad1.dpad_right && !gamepad1.dpad_left && !servo_active_for_intake) {
                servo.setPower(0);
            }
        }

        return false;
    }

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        servo.setPower(0.5);

        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    private boolean alignToTarget(double targetDegrees) {
        if(!autoAimEnabled) return true;
        if(Math.abs(targetDegrees) < 0.5) return true;

        double steeringAdjust = 0.001f * targetDegrees;

        aimFrontLeft += steeringAdjust;
        aimFrontRight -= steeringAdjust;
        aimBackLeft += steeringAdjust;
        aimBackRight -= steeringAdjust;

        double maxPower = 0.2;
        aimFrontLeft = Math.min(aimFrontLeft, maxPower);
        aimFrontRight = Math.min(aimFrontRight, maxPower);
        aimBackLeft = Math.min(aimBackLeft, maxPower);
        aimBackRight = Math.min(aimBackRight, maxPower);
        aimFrontLeft = Math.max(aimFrontLeft, -maxPower);
        aimFrontRight = Math.max(aimFrontRight, -maxPower);
        aimBackLeft = Math.max(aimBackLeft, -maxPower);
        aimBackRight = Math.max(aimBackRight, -maxPower);


        frontLeftDrive.setPower(aimFrontLeft);
        frontRightDrive.setPower(aimFrontRight);
        backLeftDrive.setPower(aimBackLeft);
        backRightDrive.setPower(aimBackRight);

        return false;
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    private boolean farPowerAuto() {
        LLResult result = limelight.getLatestResult();

        boolean aligned_to_target = false;
        boolean target_detected = false;
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                // Blue
                if(fr.getFiducialId() == 20 && Objects.equals(allianceSelected, BLUE)) {
                    target_detected = true;
                    aligned_to_target = alignToTarget(fr.getTargetXDegrees());
                }

                // Red
                if(fr.getFiducialId() == 24 && Objects.equals(allianceSelected, RED)) {
                    target_detected = true;
                    aligned_to_target = alignToTarget(fr.getTargetXDegrees() + 5.0);
                }
            }
        }

        if(!target_detected) {
            telemetry.addLine("No targets detected");
        }

        if(aligned_to_target || !target_detected) {
            aimFrontLeft = 0.0;
            aimFrontRight = 0.0;
            aimBackLeft = 0.0;
            aimBackRight = 0.0;
        }

        if(!aligned_to_target && target_detected) return true;



        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        servo.setPower(0.5);
        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 50) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }

        return false;
    }

    //Autonomous Code
//For autonomous, the robot will launch the pre-loaded 3 balls then back away from the goal, turn, and back up off the launch line.

    /**
     * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target.
     * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
     * provides a check for it to run so long as the motors are busy and the timer has not run out.
     */
    private void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        autoDriveTimer.reset();
        frontLeftDrive.setTargetPosition((int) (frontLeftDrive.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));
        frontRightDrive.setTargetPosition((int) (frontRightDrive.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));
        backLeftDrive.setTargetPosition((int) (backLeftDrive.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));
        backRightDrive.setTargetPosition((int) (backRightDrive.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        backLeftDrive.setPower(Math.abs(speed));
        backRightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {
            idle();
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Blue Alliance Autonomous
     * The robot will fire the pre-loaded balls until the 10 second timer ends.
     * Then it will back away from the goal and off the launch line.
     */
    private void doAutoBlueNear() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();

            // ((DcMotorEx) flywheel).setVelocity(firstBankVelocity);
            // Back Up
            autoDrive(0.50, -40, -40, 5000);
            // Turn
            // autoDrive(0.75, -10, 10, 5000);

            // Fire balls
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                bankShotAuto();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            ((DcMotorEx) flywheel).setVelocity(0);
            coreHex.setPower(0);
            servo.setPower(0);
            // Turn
            autoDrive(0.25, 12, -12, 5000);
            // Drive off Line
            autoDrive(0.5, -30, -30, 5000);
        }
    }

    private void doAutoBlueFar() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();

            // Back Up
            autoDrive(0.50, 5, 5, 5000);
            // Turn
            autoDrive(0.75, -6, 6, 5000);

            // Fire balls
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                farPowerAuto();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            ((DcMotorEx) flywheel).setVelocity(0);
            coreHex.setPower(0);
            servo.setPower(0);
            // Turn
            //autoDrive(0.25, 12, -12, 5000);
            // Drive off Line
            autoDrive(0.5, 13, 13, 5000);
        }
    }

    /**
     * Red Alliance Autonomous
     * The robot will fire the pre-loaded balls until the 10 second timer ends.
     * Then it will back away from the goal and off the launch line.
     */
    private void doAutoRedNear() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();

            // ((DcMotorEx) flywheel).setVelocity(firstBankVelocity);

            // Back Up
            autoDrive(0.5, -40, -40, 5000);
            // Turn
            // autoDrive(0.75, 10, -10, 5000);

            // Fire balls
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                bankShotAuto();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            ((DcMotorEx) flywheel).setVelocity(0);
            coreHex.setPower(0);
            servo.setPower(0);
            // Turn
            autoDrive(0.25, -12, 12, 5000);
            // Drive off Line
            autoDrive(0.5, -30, -30, 5000);
        }
    }
    private void doAutoRedFar() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();

            // ((DcMotorEx) flywheel).setVelocity(firstBankVelocity);

            // Back Up
            autoDrive(0.5, 5, 5, 5000);
            // Turn
            autoDrive(0.75, 6, -6, 5000);

            // Fire balls
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                farPowerAuto();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            ((DcMotorEx) flywheel).setVelocity(0);
            coreHex.setPower(0);
            servo.setPower(0);
            // Turn
            //autoDrive(0.25, -12, 12, 5000);
            // Drive off Line
            autoDrive(0.5, 13, 13, 5000);
        }
    }

}
