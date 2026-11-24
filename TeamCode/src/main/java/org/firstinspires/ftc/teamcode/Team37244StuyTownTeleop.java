package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Team37244StuyTownTeleop", group = "Testing")
public class Team37244StuyTownTeleop extends OpMode {

    // Our robot object will handle all operations with hardware.
    // This Teleop class will only manage telemetry and controller input
    Team37244Robot robot;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new Team37244Robot();

        robot.init(hardwareMap);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // === DRIVING ===
        // Reset imu Yaw if collision or disconnection causes imu error
        if (gamepad1.share) {
            robot.resetIMUYaw();
        }

        robot.fieldCentricDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        // === OTHER ===
        robot.runIntake(gamepad1.right_bumper, gamepad1.right_trigger_pressed);
        robot.runPlow(gamepad1.triangle, gamepad1.x);

        // === DRIVER STATION TELEMETRY INFO ===
        telemetry.addData("Front Drive", "left font (%.2f), right font (%.2f)", robot.getLeftFrontDrivePower(), robot.getRightFrontDrivePower());
        telemetry.addData("Back Drive", "left back (%.2f), right back (%.2f)", robot.getLeftBackDrivePower(), robot.getRightBackDrivePower());
        telemetry.addData("IMU Yaw", "robot yaw (%.2f)", robot.getIMUYaw());
        telemetry.addData("Plow State", "robot yaw (%s)", robot.getCurrentPlowState());
        telemetry.addData("Intake State", "robot yaw (%s)", robot.getCurrentIntakeState());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}