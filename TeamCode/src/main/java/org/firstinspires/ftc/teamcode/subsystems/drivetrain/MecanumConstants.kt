package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import AngularVelocity
import LinearVelocity
import kotlin.math.PI

class MecanumConstants {
    object Ids {
        const val frId: String = "frontRightMotor"
        const val flId: String = "frontLeftMotor"
        const val brId: String = "backRightMotor"
        const val blId: String = "backLeftMotor"
    }

    object Physics {
        const val ticksPerRevolution: Double = 290.0
        val circumference = Distance.fromCm(7.5 * PI)
    }

    object Velocities {
        val maxVelocityY: LinearVelocity = LinearVelocity.fromMps(1.5)
        val maxVelocityX: LinearVelocity = LinearVelocity.fromMps(13.0)
        val maxRotationVelocity: AngularVelocity = AngularVelocity.fromDegPerSec(30.0)

    }
}