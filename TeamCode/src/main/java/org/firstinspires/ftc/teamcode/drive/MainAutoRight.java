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
public class MainAutoRight extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.735;
        double stringDrop = 0.832;

        double armPicks[] = {1.98, 2.031, 2.056, 2.123, 2.156};
        double stringPicks[] = {0.695, 0.709, 0.715, 0.691, 0.678};

        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;


        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        waitForStart();

        double timeStart = getRuntime();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 68;
        // 80

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(36, -19), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(74))
                .build();
        TrajectorySequence endingStraight = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-74))
                .strafeTo(new Vector2d(36, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(12, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingStraight.end())
                .strafeTo(new Vector2d(60, -15), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        double timeout = 25;
        if (tag == 2) {
            timeout = 28;
        }

        waitForStart();

        grabbaClose();
        if (isStopRequested()) return;


        drive.followTrajectorySequenceAsync(traj1);
        double timeElap = getRuntime();
        while(opModeIsActive() && (drive.isBusy() || !armDoneFirst || !stringDone0)){
            if (getRuntime() - timeElap > 0.15) {
                armDoneFirst = armAsync(armDrop + 0.15, true,1);
                stringDone0 = stringAsync(stringDrop);
            }
            drive.update();
        }
//        drive.turn(Math.toRadians(74));
        stringMotor.setPower(0.1);

        wristDrop();

        boolean stringDoneFirst = false;
        while (opModeIsActive() && (!armDone0 || !stringDoneFirst)) {
            armDone0 = armAsync(armDrop, true, .5);
            stringDoneFirst = stringAsync(stringDrop);
        }
        grabbaOpen();
        sleep(100);

        // Cycles with cones
        for (int cone = 0; cone < 5; cone++) {
            if ((getRuntime() - timeStart) >= timeout) {
                break;
            }
            //Grab Align
            boolean armDone = false;
            boolean stringDone = false;
            wristPick();
            while (opModeIsActive() && (!armDone || !stringDone)) {
                armDone = armAsync(armPicks[cone] - 0.5, true, 1);
                stringDone = stringAsync(stringPicks[cone]);
                if ((getRuntime() - timeStart) >= timeout) {
                    break;
                }
            }

            //Grab
            boolean armDone2 = false;
            grabbaOpen();
            while (opModeIsActive() && !armDone2) {
                armDone2 = armAsync(armPicks[cone], true, 0.7);
                if ((getRuntime() - timeStart) >= timeout) {
                    returnHome();
                    break;
                }
            }
//            sleep(100);
            grabbaClose();
            sleep(300);

            boolean armDone3 = false;

            // Move to midpoint and flip wrist

            while (opModeIsActive() && !armDone3) {
                armDone3 = armAsync(armDrop + 0.8, false, 0.8);
                if ((getRuntime() - timeStart) >= timeout) {
                    returnHome();
                    break;
                }
            }
            wristDrop();

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true, 0.7);
                stringDone4 = stringAsync(stringDrop);
                if ((getRuntime() - timeStart) >= timeout) {
                    break;
                }
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
        }

//        drive.turn(Math.toRadians(-74));



        if (tag == 1) {
//            drive.turn(Math.toRadians(-74));
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingLeft);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);

            motorsStop();
            sleep(1000);
        } else if (tag == 3) {
//            drive.turn(Math.toRadians(-74));
            drive.followTrajectorySequence(endingStraight);
            drive.followTrajectory(endingRight);
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);
            motorsStop();
            sleep(1000);
        }
    }
}
