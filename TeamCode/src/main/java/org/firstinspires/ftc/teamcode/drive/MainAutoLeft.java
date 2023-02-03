package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.StarterAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class MainAutoLeft extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.748;
        double stringDrop = 1.448;

        double armPicks[] = {1.993, 2.062, 2.129, 2.145, 2.186};
        double stringPicks[] = {1.278, 1.285, 1.305, 1.325, 1.315};

        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        grabbaClose();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 68;
        // 80

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-36, -18.75), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(73))
                .build();
        TrajectorySequence endingStraight = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-73))
                .strafeTo(new Vector2d(-36, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(-12, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(-60, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();

        int tag = getAprilTag(5);
        packet.put("APRIL CONE", tag);
        dashboard.sendTelemetryPacket(packet);

        double timeout = 24.5;
        if (tag == 2) {
            timeout = 27.5;
        }

        double timeStart = getRuntime();


        drive.followTrajectorySequenceAsync(traj1);
        double timeElap = getRuntime();
        while (opModeIsActive() && (drive.isBusy() || !armDoneFirst || !stringDone0)) {
            if (getRuntime() - timeElap > 0.15) {
                armDoneFirst = armAsync(armDrop + 0.15, true, 1);
                stringDone0 = stringAsync(stringDrop);
            }
            drive.update();
        }
        stringMotor.setPower(0.1);

        wristDrop();

        boolean stringDoneFirst = false;
        while (opModeIsActive() && (!armDone0 || !stringDoneFirst)) {
            armDone0 = armAsync(armDrop, true, 1);
            stringDoneFirst = stringAsync(stringDrop);
        }
        grabbaOpen();
        sleep(200);

        // Cycles with cones
        cycleloop:
        for (int cone = 0; cone < 5; cone++) {
            if ((getRuntime() - timeStart) >= timeout) {
                break cycleloop;
            }
            //Grab Align
            boolean armDone = false;
            boolean stringDone = false;
            wristPick();
            grabbaOpen();

            while (opModeIsActive() && (!armDone || !stringDone)) {
                armDone = armAsync(armPicks[cone], true, 1);
                stringDone = stringAsync(stringPicks[cone]);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }

            sleep(100);
            grabbaClose();
            sleep(300);

            boolean armDone3 = false;

            // Move to midpoint and flip wrist

            while (opModeIsActive() && !armDone3) {
                armDone3 = armAsync(armDrop + 1, false, 1);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }
            wristDrop();

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true, 1);
                stringDone4 = stringAsync(stringDrop);
                if ((getRuntime() - timeStart) >= timeout) {
                    break cycleloop;
                }
                if (Math.abs(Math.toDegrees(imu.getAngularOrientation().secondAngle)) > 10) {
                    requestOpModeStop();
                }
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
        }

        if (armpot.getVoltage() < 0.766) {
            grabbaOpen();
        }

        wristDrop();
        returnMiddle();
//        drive.turn(Math.toRadians(-74));

        if (tag == 1) {
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingLeft);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            drive.turn(Math.toRadians(-74));
            motorsStop();
            sleep(1000);
        } else if (tag == 3) {
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingRight);
            motorsStop();
            sleep(1000);
        }
    }
}
