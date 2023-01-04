package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.StarterAuto;

@Config
@Autonomous
public class AutoStraight extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        double armDrop = 0.749;
        double stringDrop = 0.974;

        double armPicks[] = {2.041, 2.105, 2.145, 2.152, 2.23};
        double stringPicks[] = {0.641, 0.644, 0.642, 0.649, 0.658};

        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        waitForStart();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 20;

        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .strafeTo(new Vector2d(-36, -19))
                .build();
        Trajectory endingStraight = drive.trajectoryBuilder(traj1.end(), false)
                .strafeTo(new Vector2d(-36, -15))
                .build();

        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end(), false)
                .strafeTo(new Vector2d(-12, -15))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingLeft.end(), false)
                .strafeTo(new Vector2d(-60, -15))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(74));
        sleep(100); // PLACE CONES AFTER SLEEP

        // Goes and drops pre-loaded cone
        while (opModeIsActive() && !armDoneFirst) {
            armDoneFirst = armAsync(armDrop + 0.25, false);
        }

        while (opModeIsActive() && (!armDone0 || !stringDone0)) {
            armDone0 = armAsync(armDrop, true);
            stringDone0 = stringAsync(stringDrop);
        }
        grabbaOpen();

        // Cycles with cones
        for (int cone = 0; cone < 5; cone++) {

            //Grab Align
            boolean armDone = false;
            boolean stringDone = false;
            wristServo.setPosition(0);
            while (opModeIsActive() && (!armDone || !stringDone)) {
                armDone = armAsync(armPicks[cone] - 0.5, false);
                stringDone = stringAsync(stringPicks[cone]);
            }

            //Grab
            boolean armDone2 = false;
            grabbaOpen();
            while (opModeIsActive() && !armDone2) {
                armDone2 = armAsync(armPicks[cone], true);
            }
            sleep(200);
            grabbaClose();
            sleep(200);

            boolean armDone3 = false;

            while (opModeIsActive() && !armDone3) {
                armDone3 = armAsync(armDrop + 0.6, false);
            }
            wristServo.setPosition(0.94);

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true);
                stringDone4 = stringAsync(stringDrop);
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
        }

        drive.turn(Math.toRadians(-74));

        drive.followTrajectory(endingStraight);

        if (tag == 1) {
            drive.followTrajectory(endingLeft);
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);

            motorsStop();
            sleep(1000);
        } else if (tag == 3) {
            drive.followTrajectory(endingRight);
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);
            motorsStop();
            sleep(1000);
        }
    }
}
