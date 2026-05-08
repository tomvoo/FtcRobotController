
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.404)
            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.1, 0.03))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0.10617, 0.00205))
            .centripetalScaling(0);
    //.forwardZeroPowerAcceleration(-38.1)
            //.lateralZeroPowerAcceleration(-55.1);
            // .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.04))
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.0008, 0.6,0.005));



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(57.79)
            .yVelocity(43.8);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.5, 1);

    public static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(1, -3.25, Math.toRadians(-90));
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(1.19)
            .angularScalar(1.0)
            .offset(offset);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .OTOSLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
