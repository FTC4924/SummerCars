/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class TribotTeleop extends OpMode {

    double targetAngle;
    double previousAngle;
    double currentAngle;

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private IMU imu;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        targetAngle = 0;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
    @Override
    public void loop() {
        double deltaTime = timer.time();
        timer.reset();
        previousAngle = currentAngle;
        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double deltaAngle = currentAngle - previousAngle;
        telemetry.addData("angleVelocity", deltaAngle/deltaTime);
        telemetry.addData("current angle", currentAngle);

        targetAngle -= gamepad1.right_stick_x * 515 * deltaTime;
        if (Math.abs(targetAngle) > 180) {
            targetAngle = ((((targetAngle - 180) % 360) + 360) % 360) - 180;
        }
        telemetry.addData("Target Angle", targetAngle);
        final double MAX_TURN_REDUCTION = 0.14;

        final double MAX_TURN_SPEED = 0.05;

        double headingError = targetAngle - currentAngle;
        if (Math.abs(headingError) > 180) {
            headingError = ((((headingError - 180) % 360) + 360) % 360) - 180;
        }
        double headingCorrection = headingError * .03;

        double drive = gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x * MAX_TURN_SPEED; //- MAX_TURN_REDUCTION * Math.abs(drive);

        double leftPower    = Range.clip(drive - headingCorrection , -1.0, 1.0) ;
        double rightPower   = Range.clip(drive + headingCorrection , -1.0, 1.0) ;

        telemetry.addData("headingCorrection", headingCorrection);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
