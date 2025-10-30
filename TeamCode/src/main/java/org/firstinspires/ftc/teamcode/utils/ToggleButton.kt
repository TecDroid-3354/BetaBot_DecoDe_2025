package org.firstinspires.ftc.teamcode.utils

import java.util.function.BooleanSupplier

class ToggleButton(val buttonRead: BooleanSupplier) {
    private var justPressed = false

    fun wasJustPressed(): Boolean {
        val button = buttonRead.asBoolean

        if (button && !justPressed) {
            justPressed = true
            return true
        }

        justPressed = button
        return false
    }
}