package org.firstinspires.ftc.teamcode.CalibraionCamera.calibration;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

public class CameraBrightness {
    private Camera mCam = Camera.open();

    public int getBrightness(){
        int brighness;
        brighness = mCam.getParameters().getExposureCompensation();
        return brighness;
    }
    public boolean setBrightness(int brightness){
        if(brightness < mCam.getParameters().getMaxExposureCompensation() && brightness > mCam.getParameters().getMinExposureCompensation()){
            mCam.getParameters().setExposureCompensation(brightness);
            return true;
        }
        else{
            return false;
        }
    }

}
