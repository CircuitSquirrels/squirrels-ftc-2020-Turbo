import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Utilities.Json;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;

public class AshleysUnfaithInMyAbilities {

    Json json;

    @Before
    public void init() {
        json = new Json();
    }

    @Test
    public void start() {
        try {
            JSONObject object = new JSONObject();
            object.put("Waypoint", json.createNav2D(new MecanumNavigation.Navigation2D(10,0,-30)));

            json.saveData(AppUtil.getInstance().getSettingsFile("./TestData/file.json").getAbsolutePath(), object);

            object = new JSONObject(json.getFileContents("./TestData/file.json"));
            System.out.println(object.toString(3));

            System.out.println(json.getNav2D(object.getJSONObject("Waypoint")));
        } catch (JSONException | IOException e) {
            e.printStackTrace();
        }
    }
}
