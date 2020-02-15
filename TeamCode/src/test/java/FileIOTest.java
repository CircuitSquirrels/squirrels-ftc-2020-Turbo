import android.util.JsonWriter;

import org.firstinspires.ftc.teamcode.DeadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryTicks;
import org.firstinspires.ftc.teamcode.FileIO.JsonUtil;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.Serializable;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class FileIOTest {

    //https://blog.codota.com/how-to-convert-a-java-object-into-a-json-string/
    //https://medium.com/@naimishverma50/android-writing-a-file-as-a-json-object-400131f6063b
    //https://gist.github.com/whitfin/8676531

    @Before
    public void initialize() {

    }

    @Test
    public void createJsonString() {
        System.out.println("Create json string from object");
        Navigation2D waypoint = new Navigation2D(1,2,0);

        String json = JsonUtil.toJson(waypoint);
        System.out.println(json);


//
//
//
//    try {
//        JSONObject navPoint = new JSONObject(waypoint.toString());
//    } catch (JSONException e) {
//        e.printStackTrace();
//    }





        TestBean testBean = new TestBean(10,20,0);




//        JsonWriter jsonWriter = new JsonWriter();

    }

    public void testFileIO() {

        String FILE_NAME = "file-name";
//        File file = new File(this.getFilesDir(), FILE_NAME);
    }


    static class TestBean implements Serializable {
        private double x,y,theta;

        public TestBean() {
        x=0;
        y=0;
        theta=0;
        }

        public TestBean(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public void setX(double x) {
            this.x = x;
        }

        public double getX() {
            return x;
        }

        public void setY(double y) {
            this.y = y;
        }

        public double getY() {
            return y;
        }

        public void setTheta(double theta) {
            this.theta = theta;
        }

        public double getTheta() {
            return theta;
        }
    }

}
