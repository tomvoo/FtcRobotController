package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "DecodeAutoOpMode", group = "PedroPathing")
public class DecodeAutoOpMode extends OpMode {

        private Follower follower;

        private Limelight3A limelight;
        // Q (Process noise/drift) is low because OTOS is smooth. R (Measurement noise)
        // is high to heavily filter Limelight jitter.
        private KalmanFilter kfX = new KalmanFilter(0.05, 0.5);
        private KalmanFilter kfY = new KalmanFilter(0.05, 0.5);
        private KalmanFilter kfH = new KalmanFilter(0.05, 0.5);
        private double lastTa = 0;
        private Pose previousPose;

        private RobotHardware robot = new RobotHardware();
        private Paths paths;
        private int pathState = 0;
        private Timer pathTimer = new Timer();

        private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
        private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
        private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

        private PathChain triangle;

        @Override
        public void init() {
                robot.init(hardwareMap);

                follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
                paths = new Paths(follower);

                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.pipelineSwitch(0);
                limelight.start();

                Drawing.init();
                previousPose = new Pose(startPose.getX(), startPose.getY(), startPose.getHeading());
        }

        @Override
        public void init_loop() {
                Drawing.drawDebug(follower);
                updateVision();
                telemetry.update();
        }

        public static class Paths {
                public PathChain Start2InitialShoot;
                public PathChain RedStart2Spike1ToShoot;
                public PathChain RedShoot2Spike2ToShoot;
                public PathChain Leave;

                public Paths(Follower follower) {
                        Start2InitialShoot = follower.pathBuilder()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(56.000, 8.000),
                                                                        new Pose(56.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                                        .build();

                        RedStart2Spike1ToShoot = follower.pathBuilder()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(56.000, 12.000),
                                                                        new Pose(56.000, 38.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(56.000, 38.000),
                                                                        new Pose(20.000, 38.000)))
                                        .setTangentHeadingInterpolation()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(20.000, 38.000),
                                                                        new Pose(56.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                                        .build();

                        RedShoot2Spike2ToShoot = follower.pathBuilder()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(56.000, 12.000),
                                                                        new Pose(55.921, 62.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(55.921, 62.000),
                                                                        new Pose(20.000, 62.000)))
                                        .setTangentHeadingInterpolation()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(20.000, 62.000),
                                                                        new Pose(56.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                                        .build();

                        Leave = follower.pathBuilder()
                                        .addPath(
                                                        new BezierLine(
                                                                        new Pose(56.000, 12.000),
                                                                        new Pose(46.000, 24.000)))
                                        .setConstantHeadingInterpolation(Math.toRadians(135))
                                        .build();

                }
        }

        @Override
        public void start() {
                setPathState(0);
        }

        private void updateVision() {
                // Calculate the delta in movement from Pedro tracking to apply to process noise
                Pose currentPose = follower.getPose();
                double dx = currentPose.getX() - previousPose.getX();
                double dy = currentPose.getY() - previousPose.getY();
                double dh = currentPose.getHeading() - previousPose.getHeading();

                // Prediction Step 2: Progress the Kalman error matrices (drift occurs relative
                // to distance moved)
                kfX.predict(dx);
                kfY.predict(dy);
                kfH.predict(dh);

                // Pass Pedro's internal heading to the Limelight to improve Megatag 2 accuracy
                // limelight.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading()) - 90.0);

                LLResult result = limelight.getLatestResult();
                // Check for validity and confirm it's a new vision frame by comparing Target
                // Area (Ta)
                if (result != null && result.isValid() && result.getTa() != lastTa) {
                        lastTa = result.getTa(); // Register fresh tracking data

                        Pose3D botpose = result.getBotpose();
                        //Pose3D botpose = result.getBotpose_MT2();
                        if (botpose != null) {
                                Pose ftcPose = new Pose(botpose.getPosition().x, botpose.getPosition().y,
                                                botpose.getOrientation().getYaw(AngleUnit.RADIANS));
                                Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                                // Measurement Step: Fuse the robust Pedro state with the absolute but noisy
                                // vision data
                                double filteredX = kfX.update(follower.getPose().getX(), pedroPose.getX());
                                double filteredY = kfY.update(follower.getPose().getY(), pedroPose.getY());
                                double filteredH = kfH.update(follower.getPose().getHeading(), pedroPose.getHeading());

                                // Update Pedro Pathing with the smooth filtered pose
                                follower.setPose(new Pose(filteredX, filteredY, filteredH));
                                follower.update();

                                telemetry.addData("pedropose X",ftcPose.getX());
                                telemetry.addData("follower x", follower.getPose().getX());
                        }
                }


                previousPose = new Pose(follower.getPose().getX(), follower.getPose().getY(),
                                follower.getPose().getHeading());
        }

        public void setPathState(int state) {
                pathState = state;
                pathTimer.resetTimer();
        }

        public void autonomousPathUpdate() {
                switch (pathState) {
                        case 0:
                                // Start flywheel aggressively and push the robot to the firing position
                                robot.flywheel.setVelocity(1625);
                                follower.followPath(paths.Start2InitialShoot, 1.0, true);
                                setPathState(1);
                                break;

                        case 1:
                                robot.flywheel.setVelocity(1625); // keep running
                                // Wait until we arrive at the initial shooting position
                                if (!follower.isBusy()) {
                                        setPathState(2);
                                }
                                break;

                        case 2:
                                // Spin flywheel up & Shoot
                                robot.flywheel.setVelocity(1625);
                                // Shoot if velocity is reached
                                if (Math.abs(robot.flywheel.getVelocity()) >= 1625 - 50) {
                                        robot.coreHex.setPower(1);
                                } else {
                                        robot.coreHex.setPower(0);
                                }

                                // After 6 seconds, transition
                                if (pathTimer.getElapsedTimeSeconds() > 6) {
                                        robot.coreHex.setPower(0);
                                        follower.followPath(paths.RedStart2Spike1ToShoot, 1.0, true);

                                        robot.intake.setVelocity(6000);
                                        robot.lifter.setPosition(1.0);
                                        robot.servo.setPower(1);

                                        setPathState(3);
                                }
                                break;

                        case 3:
                                robot.flywheel.setVelocity(1625); // keep running

                                // When we reach the return-path sequence (index 2 for 3rd BezierLine), turn off
                                // intake
                                if (follower.getCurrentPathNumber() >= 2) {
                                        robot.intake.setVelocity(0);
                                        robot.lifter.setPosition(0.25);
                                        robot.servo.setPower(0);
                                }

                                if (!follower.isBusy()) {
                                        setPathState(4);
                                }
                                break;

                        case 4:
                                // Shoot second batch
                                robot.flywheel.setVelocity(1625);
                                if (Math.abs(robot.flywheel.getVelocity()) >= 1625 - 50) {
                                        robot.coreHex.setPower(1);
                                } else {
                                        robot.coreHex.setPower(0);
                                }

                                // After 6 seconds, transition
                                if (pathTimer.getElapsedTimeSeconds() > 6) {
                                        robot.coreHex.setPower(0);
                                        follower.followPath(paths.RedShoot2Spike2ToShoot, 1.0, true);

                                        robot.intake.setVelocity(6000);
                                        robot.lifter.setPosition(1.0);
                                        robot.servo.setPower(1);

                                        setPathState(5);
                                }
                                break;

                        case 5:
                                robot.flywheel.setVelocity(1625);

                                // When we reach the return-path sequence
                                if (follower.getCurrentPathNumber() >= 2) {
                                        robot.intake.setVelocity(0);
                                        robot.lifter.setPosition(0.25);
                                        robot.servo.setPower(0);
                                }

                                if (!follower.isBusy()) {
                                        setPathState(6);
                                }
                                break;

                        case 6:
                                // Final Shoot
                                robot.flywheel.setVelocity(1625);
                                if (Math.abs(robot.flywheel.getVelocity()) >= 1625 - 50) {
                                        robot.coreHex.setPower(1);
                                } else {
                                        robot.coreHex.setPower(0);
                                }

                                // Finish up
                                if (pathTimer.getElapsedTimeSeconds() > 6) {
                                        robot.coreHex.setPower(0);
                                        robot.flywheel.setVelocity(0);
                                        follower.followPath(paths.Leave, 1.0, true);
                                        setPathState(7);
                                }
                                break;

                        case 7:
                                if (!follower.isBusy()) {
                                        setPathState(-1);
                                }
                                break;
                }
        }

        @Override
        public void loop() {
                // Prediction Step 1: Let Pedro naturally advance its odometry tracking
                follower.update();
                Drawing.drawDebug(follower);

                updateVision();
                autonomousPathUpdate();

                telemetry.update();
        }
}
