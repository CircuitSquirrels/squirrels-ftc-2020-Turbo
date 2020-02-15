package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.stream.Collectors;

public class Json {

    public String getFileContents(String path) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(path));
        return reader.lines().collect(Collectors.joining(System.lineSeparator()));
    }

    public boolean saveData(String path, JSONObject object) {
        try {
            FileWriter fileWriter;
            fileWriter = new FileWriter(path);
            fileWriter.write(object.toString(3));
            fileWriter.close();
            return true;
        } catch (IOException | JSONException e) {
            Log.w("Error", "Failed to save data... " + e.getMessage());
        }
        return false;
    }

    public MecanumNavigation.Navigation2D getNav2D(JSONObject object) {
        try {
            return new MecanumNavigation.Navigation2D(object.getDouble("x"), object.getDouble("y"), object.getDouble("theta"));
        } catch (JSONException e) {
            return new MecanumNavigation.Navigation2D(0,0,0);
        }
    }

    public JSONObject createNav2D(MecanumNavigation.Navigation2D nav2d) throws JSONException {
        JSONObject obj = new JSONObject();
        obj.put("x", nav2d.x);
        obj.put("y", nav2d.y);
        obj.put("theta", nav2d.theta);
        return obj;
    }
}
