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

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Objects;


@TeleOp(name="DecodeOpModeWithDash", group="Linear OpMode")
public class DecodeOpModeDash extends LinearOpMode {

    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    // Setting our velocity targets. These values are in ticks per second!
    public static int bankVelocity = 1350; // 1350 * 3
    public static int firstBankVelocity = 1350; // 1350 * 3
    public static int farVelocity = 1625; // 1625 * 3
    public static final int maxVelocity = 1625; // 1625 * 3

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
        // Initialize the hardware variables from the RobotHardware class
        robot.init(hardwareMap);

        // On initialization the Driver Station will prompt for which OpMode should be run
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
        while (opModeIsActive()) {
            if(gamepad1.circleWasPressed()) {
                intake_active = !intake_active;
            }

            if(intake_active) {
                robot.intake.setVelocity(6000);
                robot.lifter.setPosition(1.0);
                servo_active_for_intake = true;
            } else {
                robot.intake.setVelocity(0);
                robot.lifter.setPosition(0.25);
                servo_active_for_intake = false;
            }

            if(gamepad1.psWasPressed()) {
                reduction_active = !reduction_active;
            }

            double reduction = reduction_active ? 0.4 : 1.0;
            double axial = reduction * -gamepad1.left_stick_y;
            double lateral = reduction * gamepad1.left_stick_x;
            double yaw = reduction * gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            boolean autoAimActive = setFlywheelVelocity();

            if(!autoAimActive) {
                robot.setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            }

            manualCoreHexAndServoControl();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            panelsTelemetry.addData("Flywheel Velocity", robot.flywheel.getVelocity());
            panelsTelemetry.addData("Flywheel Power", robot.flywheel.getPower());
            telemetry.addData("Reduction Active", reduction_active);
            // telemetry.update();
            panelsTelemetry.update(telemetry);


        }
    }

    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP)) state = AUTO_BLUE_FAR;
            else if (state.equals(AUTO_BLUE_FAR)) state = AUTO_BLUE_NEAR;
            else if (state.equals(AUTO_BLUE_NEAR)) state = AUTO_RED_FAR;
            else if (state.equals(AUTO_RED_FAR)) state = AUTO_RED_NEAR;
            else if (state.equals(AUTO_RED_NEAR)) state = TELEOP;
        }
        telemetry.addLine("Press Home Button to cycle options");
        telemetry.addData("CURRENT SELECTION", state);
        if (!state.equals(TELEOP)) telemetry.addLine("Please remember to enable the AUTO timer!");
        telemetry.addLine("Press START to start your program");
        return state;
    }

    private String selectAlliance(String state, boolean cycleNext) {
        if (cycleNext) {
            state = state.equals(RED) ? BLUE : RED;
        }
        telemetry.addLine("Press X Button to cycle alliance");
        telemetry.addData("CURRENT ALLIANCE", state);
       return state;
    }

    private void manualCoreHexAndServoControl() {
        if (gamepad1.cross) robot.coreHex.setPower(0.5);
        else if (gamepad1.triangle) robot.coreHex.setPower(-0.5);

        if (servo_active_for_intake) robot.servo.setPower(1);
        else if (gamepad1.dpad_left) robot.servo.setPower(-1);
        else if (gamepad1.dpad_right) robot.servo.setPower(1);
    }

    private boolean setFlywheelVelocity() {
        if (gamepad1.options) {
            robot.flywheel.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            return farPowerAuto();
        } else if (gamepad1.right_bumper) {
            bankShotAuto();
        } else if (gamepad1.square) {
            robot.flywheel.setVelocity(maxVelocity);
        } else {
            robot.flywheel.setVelocity(0);
            robot.coreHex.setPower(0);
            if (!gamepad1.dpad_right && !gamepad1.dpad_left && !servo_active_for_intake) {
                robot.servo.setPower(0);
            }
        }
        return false;
    }

    private void bankShotAuto() {
        robot.flywheel.setVelocity(bankVelocity);
        robot.servo.setPower(0.5);
        robot.coreHex.setPower(robot.flywheel.getVelocity() >= bankVelocity - 50 ? 1 : 0);
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
        aimFrontLeft = Math.max(-maxPower, Math.min(aimFrontLeft, maxPower));
        aimFrontRight = Math.max(-maxPower, Math.min(aimFrontRight, maxPower));
        aimBackLeft = Math.max(-maxPower, Math.min(aimBackLeft, maxPower));
        aimBackRight = Math.max(-maxPower, Math.min(aimBackRight, maxPower));

        robot.setDrivePower(aimFrontLeft, aimFrontRight, aimBackLeft, aimBackRight);
        return false;
    }

    private boolean farPowerAuto() {
        LLResult result = robot.limelight.getLatestResult();
        boolean aligned_to_target = false;
        boolean target_detected = false;

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if(fr.getFiducialId() == 20 && Objects.equals(allianceSelected, BLUE)) {
                    target_detected = true;
                    aligned_to_target = alignToTarget(fr.getTargetXDegrees());
                }
                if(fr.getFiducialId() == 24 && Objects.equals(allianceSelected, RED)) {
                    target_detected = true;
                    aligned_to_target = alignToTarget(fr.getTargetXDegrees() + 5.0);
                }
            }
        }

        if(!target_detected) telemetry.addLine("No targets detected");

        if(aligned_to_target || !target_detected) {
            aimFrontLeft = aimFrontRight = aimBackLeft = aimBackRight = 0.0;
        }

        if(!aligned_to_target && target_detected) return true;

        robot.flywheel.setVelocity(farVelocity);
        robot.servo.setPower(0.5);
        robot.coreHex.setPower(robot.flywheel.getVelocity() >= farVelocity - 50 ? 1 : 0);
        return false;
    }

    private void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        autoDriveTimer.reset();
        int leftTarget = (int) (robot.frontLeftDrive.getCurrentPosition() + leftDistanceInch * RobotHardware.WHEELS_INCHES_TO_TICKS);
        int rightTarget = (int) (robot.frontRightDrive.getCurrentPosition() + rightDistanceInch * RobotHardware.WHEELS_INCHES_TO_TICKS);
        
        robot.setDriveTargetPosition(leftTarget, rightTarget, leftTarget, rightTarget);
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDrivePower(Math.abs(speed));

        while (opModeIsActive() && robot.isDriveBusy() && autoDriveTimer.milliseconds() < timeout_ms) {
            idle();
        }
        robot.setDrivePower(0);
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void doAutoBlueNear() {
        if (opModeIsActive()) {
            autoDrive(0.50, -40, -40, 5000);
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                bankShotAuto();
            }
            stopRobot();
            autoDrive(0.25, 12, -12, 5000);
            autoDrive(0.5, -30, -30, 5000);
        }
    }

    private void doAutoBlueFar() {
        if (opModeIsActive()) {
            autoDrive(0.50, 5, 5, 5000);
            autoDrive(0.75, -6, 6, 5000);
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                farPowerAuto();
            }
            stopRobot();
            autoDrive(0.5, 13, 13, 5000);
        }
    }

    private void doAutoRedNear() {
        if (opModeIsActive()) {
            autoDrive(0.5, -40, -40, 5000);
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                bankShotAuto();
            }
            stopRobot();
            autoDrive(0.25, -12, 12, 5000);
            autoDrive(0.5, -30, -30, 5000);
        }
    }

    private void doAutoRedFar() {
        if (opModeIsActive()) {
            autoDrive(0.5, 5, 5, 5000);
            autoDrive(0.75, 6, -6, 5000);
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                farPowerAuto();
            }
            stopRobot();
            autoDrive(0.5, 13, 13, 5000);
        }
    }

    private void stopRobot() {
        robot.flywheel.setVelocity(0);
        robot.coreHex.setPower(0);
        robot.servo.setPower(0);
    }
}
