package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class StickyGamepadGenerator{
    Gamepad lastState = new Gamepad();

    public static int startIndex = 36;
    public static int byteNo = 4;

    public Gamepad update(Gamepad uGamepad){
        byte[] uGamepadByteArray = uGamepad.toByteArray();
        byte[] lastStateByteArray = uGamepad.toByteArray();
        byte[] stickyButtonsByteArray = uGamepadByteArray;

        for(int i = startIndex;i<startIndex + byteNo;i++) stickyButtonsByteArray[i] = (byte) (uGamepadByteArray[i] & (lastStateByteArray[i] ^ (1<<4)));

        Gamepad aux = new Gamepad();
        aux.fromByteArray(stickyButtonsByteArray);
        return aux;
    }


}
