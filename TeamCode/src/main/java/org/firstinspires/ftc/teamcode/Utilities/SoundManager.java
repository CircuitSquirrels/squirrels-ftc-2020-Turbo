package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;


/**
 *      Based on sample provided by FTC.
 *
 *      Any sounds you wish to play should be in your path as follows:
 *          <project root>/TeamCode/src/main/res/raw
 *
 *      Copy any .wav files you want to play into this folder.
 *      Make sure that your files ONLY use lower-case characters, and have no spaces or special characters other than underscore.
 *
 *      The name you give your .wav files will become the resource ID for these sounds.
 *      eg:  gold.wav becomes R.raw.gold
 *
 *      Use the SoundManager as follows:
 *      Construct the object:
 *      eg: SoundManger soundManager = new SoundManager(this);
 *          where this refers to a OpMode class
 *      eg: soundManager.addSound("gold");
 *      where "gold.wav" is in the /TeamCode/src/main/res/raw folder*
 *      use addSound() to add all files required.
 *      If a sound cannot be found at this stage, the method will return false, and
 *      telemetry will be shown indicating that the file was not found.
 *
 *      Preload sounds:
 *      eg: soundManager.preloadAllSounds();
 *          This will provide telemetry showing which sounds were found.
 *
 *      Play sound:
 *      eg: soundManager.play("gold");
 *
 */
public class SoundManager {

    OpMode opMode;
    Map<String,Integer> sound_ID_map = new HashMap<>();


    public SoundManager(OpMode opMode) {
        this.opMode = opMode;
    }

    // input sound names, store their ID in HashMap
    public boolean addSound(String soundName) {
        int soundID = opMode.hardwareMap.appContext.getResources().
                getIdentifier(soundName, "raw", opMode.hardwareMap.appContext.getPackageName());
        if(soundID != 0) {
            sound_ID_map.put(soundName,soundID);
            return true;
        } else {
            // Sound not found
            opMode.telemetry.addData(soundName, "Not Found.");
            return false;
        }

    }


    // Initialize, preload all sound IDs and display telemetry for their status
    public void preloadAllSounds() {
        boolean soundInitialized = false;
        int soundID;
        Set<String> keySet = sound_ID_map.keySet();
        String[] keyList = keySet.toArray(new String[keySet.size()]);
        for(String soundKeyName : keyList) {
            soundID = sound_ID_map.get(soundKeyName);
            soundInitialized = SoundPlayer.getInstance().preload(opMode.hardwareMap.appContext, soundID);
            if(soundInitialized) {
                opMode.telemetry.addData(soundKeyName, "Initialized.");
            } else {
                opMode.telemetry.addData(soundKeyName, "Not Found.");
            }
        }
    }

    // Play sounds on request
    public void play(String soundName){
        if (sound_ID_map.containsKey(soundName)) {
            try {
                int soundID = sound_ID_map.get(soundName);
                SoundPlayer.getInstance().startPlaying(opMode.hardwareMap.appContext, soundID);
            } catch (Exception e) {
                opMode.telemetry.addData("Error:", "cannot play sound \"" + soundName + "\"");
            }
        } else {
            opMode.telemetry.addData("Error:", "Sound file \"" + soundName + "\""
                    + " has not been added with AddSound().");
        }
    }

}
