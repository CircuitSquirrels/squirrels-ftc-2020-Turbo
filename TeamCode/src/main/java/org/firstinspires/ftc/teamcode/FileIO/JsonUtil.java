package org.firstinspires.ftc.teamcode.FileIO;

import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.json.JSONException;
import org.json.JSONObject;

//https://www.javacodegeeks.com/2013/10/android-json-tutorial-create-and-parse-json-data.html
public class JsonUtil {

    public static String toJson(MecanumNavigation.Navigation2D navigation2D) {
        try {
            JSONObject jObject = new JSONObject();
//            jObject.put("nav2D",navigation2D);
            jObject.put("x", navigation2D.x);
            jObject.put("y", navigation2D.y);
            jObject.put("theta", navigation2D.theta);
            return jObject.toString();

        } catch (JSONException e) {
            e.printStackTrace();
        }

        return null; // Didn't convert correctly.
    }

}
