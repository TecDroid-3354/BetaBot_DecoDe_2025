package org.firstinspires.ftc.teamcode.shooter

import com.seattlesolvers.solverslib.controller.PIDController
import com.seattlesolvers.solverslib.hardware.motors.Motor

object ShooterConstants {
    val shooterMotorId = "shooterMotor"
    val isMotorInverted = false
    val zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
    val direction = Motor.Direction.FORWARD
    // This value needs to be measured physically
    val ticksPerRevolution = 28.0
    val pidController = PIDController(0.1, 0.0, 0.0)
}