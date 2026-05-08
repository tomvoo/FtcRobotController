package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    /* Declare OpMode members. */
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotorEx flywheel = null;
    public DcMotor coreHex = null;
    public DcMotorEx intake = null;
    public Servo lifter = null;
    public CRServo servo = null;
    public Limelight3A limelight = null;

    /* Constants */
    public static final double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        coreHex = hwMap.get(DcMotor.class, "coreHex");
        intake = hwMap.get(DcMotorEx.class, "intake");

        // Define and Initialize Servos
        servo = hwMap.get(CRServo.class, "servo");
        lifter = hwMap.get(Servo.class, "lifter");

        // Define and Initialize Sensors
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        setDrivePower(0);
        flywheel.setPower(0);
        coreHex.setPower(0);
        intake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up Flywheel PIDF
        PIDFCoefficients pidfNew = new PIDFCoefficients(45.0, 0.0, 0.0, 15.0);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Initialize Servos
        servo.setPower(0);
        lifter.setPosition(0.25);

        // Initialize Limelight
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setDrivePower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    public void setDriveTargetPosition(int fl, int fr, int bl, int br) {
        frontLeftDrive.setTargetPosition(fl);
        frontRightDrive.setTargetPosition(fr);
        backLeftDrive.setTargetPosition(bl);
        backRightDrive.setTargetPosition(br);
    }

    public boolean isDriveBusy() {
        return frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy();
    }
}
