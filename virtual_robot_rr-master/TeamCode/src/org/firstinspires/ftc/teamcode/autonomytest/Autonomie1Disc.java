package org.firstinspires.ftc.teamcode.autonomytest;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.autonomytest.AutonomieTest1;

@Autonomous(name = "Autonomie1Disc", group = "autonomytest")
public class Autonomie1Disc extends OpMode {

    Robot bot;

    private enum State {BEGIN, AWAY, PAUSE, PAUSE1, PAUSE2, PAUSE3, PAUSE4, PAUSE5, PAUSE6, PAUSE7, AWAY1, AWAY2, AWAY3, AWAY4, AWAY5, AWAY6, AWAY7, AWAY8, DONE,}
    State state = State.BEGIN;

    NanoClock clock;
    double startPause;

    Trajectory traj;

    public void init(){
        bot = new Robot(hardwareMap);
        clock = NanoClock.system();
    }

    public void loop() {
        bot.updatePoseEstimate();
        Pose2d currentPose = bot.getPoseEstimate();

        telemetry.addData("POSE", "X = %.2f  Y = %.2f  H = %.1f", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        switch (state) {
            case BEGIN:
                state = State.AWAY;
                traj = new TrajectoryBuilder(new Pose2d(), bot.constraints)
                        .strafeLeft(5)
                        .splineToConstantHeading(new Vector2d(63,26),0)
                        .build();
                bot.follower.followTrajectory(traj);
                break;
            case AWAY:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca primul disc in powershot
                }
                break;
            case PAUSE:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY1;
                    traj = new TrajectoryBuilder(new Pose2d(63, 26, 0), bot.constraints)
                            .strafeTo(new Vector2d(63, 22))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY1:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE1;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca al doilea disc in powershot
                }
                break;
            case PAUSE1:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY2;
                    traj = new TrajectoryBuilder(new Pose2d(63, 22, 0), bot.constraints)
                            .strafeTo(new Vector2d(63, 18))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY2:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE2;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca al treilea disc in powershot
                }
                break;
            case PAUSE2:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY3;
                    traj = new TrajectoryBuilder(new Pose2d(63, 18, 0), bot.constraints)
                            .strafeTo(new Vector2d(85,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY3:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE3;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //lasa wobble1
                }
                break;
            case PAUSE3:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY4;
                    traj = new TrajectoryBuilder(new Pose2d(85, 0, 0), bot.constraints)
                            .strafeRight(10)
                            .splineToConstantHeading(new Vector2d(15,-22.5),0)
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY4:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE4;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //ia wobble2
                }
                break;
            case PAUSE4:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY5;
                    traj = new TrajectoryBuilder(new Pose2d(15, -22.5, 0), bot.constraints)
                            .strafeTo(new Vector2d(10,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY5:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE5;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //lasa wobble2 si totodata parcheaza
                }
                break;
            case PAUSE5:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY6;
                    traj = new TrajectoryBuilder(new Pose2d(10, 0, 0), bot.constraints)
                            .lineTo(new Vector2d(63,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY6:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE6;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca dicul pe care il ia.
                }
                break;
            case PAUSE6:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY7;
                    traj = new TrajectoryBuilder(new Pose2d(63, 0, 0), bot.constraints)
                            .lineTo(new Vector2d(80,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY7:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE7;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca dicul pe care il ia.
                }
                break;
            case PAUSE7:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY8;
                    traj = new TrajectoryBuilder(new Pose2d(80, 0, 0), bot.constraints)
                            .lineTo(new Vector2d(70,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY8:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.DONE;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                    //arunca dicul pe care il ia.
                }
                break;
            case DONE:
                break;
        }
    }

}
