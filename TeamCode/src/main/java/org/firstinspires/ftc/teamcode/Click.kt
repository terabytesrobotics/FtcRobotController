package org.firstinspires.ftc.teamcode

class Click() {

    var xAlready: Boolean = false
    fun oneClick(x: Boolean): Boolean {

        if (x && !xAlready) {
            return true
            xAlready = true
        } else if (!x) {
            xAlready = false
            return false
        } else {
            return false
        }
    }
}
