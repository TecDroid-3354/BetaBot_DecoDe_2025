package org.firstinspires.ftc.teamcode.subsystems.drivetrain

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
}