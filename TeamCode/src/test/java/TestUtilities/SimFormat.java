package TestUtilities;

import java.text.DecimalFormat;

public class SimFormat {

    public static boolean isDisplayInterval(int displaysPerSecond, double simTime, double simulationTimeStep) {
        int simStep = (int) Math.floor( simTime/simulationTimeStep );
        int stepsPerSecond = (int) Math.floor(1/simulationTimeStep);
        int stepsPerDisplay = stepsPerSecond / displaysPerSecond;
        return (simStep % stepsPerDisplay) == 0;
    }

    public static String padStringTo(int desiredLength, String currentString) {
        return currentString.concat(getPaddingString(desiredLength,currentString));
    }

    /**
     * Centered within padding.
     * @param desiredLength
     * @param currentString
     * @return
     */
    public static String padCenteredStringTo(int desiredLength, String currentString) {
        StringBuffer paddingString = new StringBuffer(getPaddingString(desiredLength,currentString));
        paddingString.insert(paddingString.length()/2,currentString);
        return paddingString.toString();
    }

    public static String getPaddingString(int desiredLength, String currentString) {
        return getPaddingString(desiredLength,currentString.length());
    }

    public static String getPaddingString(int desiredLength, int currentLength) {
        String padding = new String("");
        if (desiredLength > currentLength) {
            for(int i = 0; i < desiredLength - currentLength; ++i) {
                padding = padding.concat(" ");
            }
        }
        return padding;
    }

    public static DecimalFormat df = new DecimalFormat("0.00");
    public static DecimalFormat df_prec = new DecimalFormat(("0.00000000"));

    public static class SimulationTime {
        private double time = 0.0;

        public void setTime(double time) {
            this.time = time;
        }

        public void incrementTime(double timeStep) {
            this.time += timeStep;
        }

        public double time() {
            return time;
        }
    }

}
