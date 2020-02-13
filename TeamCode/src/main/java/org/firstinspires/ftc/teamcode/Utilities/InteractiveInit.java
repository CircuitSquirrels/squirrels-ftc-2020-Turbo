package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.ListIterator;

/**
 * Created by Stephen on 2/11/17.
 */

public class InteractiveInit {

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Controller controller;


    private boolean unlocked = true;
    private ArrayList<VarOption> options = new ArrayList<>();
    private int cursor_location = 0;

    public InteractiveInit(RobotHardware opmode) {
        this.telemetry = opmode.telemetry;
        this.gamepad1 = opmode.gamepad1;
        this.controller = new Controller(this.gamepad1);
    }

    /**
     *      Applies the selected value to the Mutables.
     */
    private void apply() {
        for (VarOption arg : options) {
            arg.apply();
        }
    }

    /**
     *      Go to the next variable listed on the menu.
     */
    private void nextOption() {
        if (cursor_location < options.size()) {
            options.get(cursor_location).next();
        }
    }

    /**
     *      Go to the previous variable listed on the menu.
     */
    private void prevOption() {
        if (cursor_location < options.size()) {
            options.get(cursor_location).prev();
        }
    }

    /**
     *              Add a new option to the interactive menu.
     *
     * @param var   Mutable variable that will be assigned the value selected on the menu.
     * @param name  Displayed name of the variable.
     * @param args  All the values that can be chosen from on the menu.
     * @param <T>   Generic type that allows the method to accept most data types.
     * @return      Returns instance of the object that can be used to update the value or append further.
     */
    public <T> InteractiveInit addOption(Mutable<T> var, String name, T... args) {
        options.add(new VarOption<>(var, name, args));
        return this;
    }

    /**
     *          Check whether or not any options have been added.
     *
     * @return  Return whether or not the options array is empty.
     */
    public boolean isEmpty() {
        return options.isEmpty();
    }

    /**
     *      Get the amount of options.
     *
     *      @return The amount of options registered in the array.
     */
    private int size() {
        return options.size();
    }


    private void displayMenu() {
        String margin = "       ";
        String cursor = " >> ";
        int n = size();

        String[] cursorMargin = new String[n];
        Arrays.fill(cursorMargin, " ");

        if(unlocked) {
            Arrays.fill(cursorMargin, margin);
            for (int i = 0; i < n; ++i) {
                if (cursor_location == i) {
                    cursorMargin[i] = cursor;
                }
            }
        }


        // Return if no interactive init options have been added
//        if(this.isEmpty()) return;

        telemetry.addLine(unlocked ? "--- MENU OPTIONS ---" : "--- MENU LOCKED ---");
        for(int i = 0; i < options.size(); ++i) {
            telemetry.addData(cursorMargin[i] + options.get(i), options.get(i).selected().toString());
        }

        telemetry.addLine("--- MENU CONTROLS ---");
        if(unlocked) {
            telemetry.addData("To edit: ","Use Direction Pad");
            telemetry.addData("To lock: ","Press  A button");
        } else {
            telemetry.addData("To lock: ","Press  A button");
        }
    }

    // Updates the menu display
    public void update() {
        displayMenu();
        telemetry.update();
        updateInputs(); // Inputs active even when locked, to allow unlocking.
    }

    // Lock our selection and apply our selected settings
    public void lock() {
        apply();
        unlocked = false;
        displayMenu(); // Display 'locked' version of menu
        telemetry.update();
    }

    private void unlock() {
        unlocked = true;
    }

    /*
     * Take gamepad inputs and
     * modify parameters accordingly.
     */
    private void updateInputs() {
        // Inputs are updated using the gamepad controls.
        controller.update();

        if(gamepad1 == null) {
            lock();
            return;
        }

        if (unlocked) {
            if (controller.dpadDownOnce()) {
                ++cursor_location;
                if (cursor_location >= size())
                    cursor_location = size() - 1;
            } else if (controller.dpadUpOnce()) {
                --cursor_location;
                if (cursor_location < 0)
                    cursor_location = 0;
            } else if (controller.dpadRightOnce()) {
                nextOption();
            } else if (controller.dpadLeftOnce()) {
                prevOption();
            } else if (controller.AOnce()) {
                lock();
            }
        } else {
            if (controller.BOnce()) {
                unlock();
            }
        }
    }
}


class VarOption<T> {
    private T value;
    private ListIterator<T> current;
    private ArrayList<T> values = new ArrayList<>();
    private String name;
    private Mutable<T> var;

    VarOption(Mutable<T> variable, String name, T... args) {
        var = variable;
        this.name = name;

        Collections.addAll(values, args);

        // Set the displayed value to the current state of the passed Mutable
        current = values.listIterator();
        while (current.hasNext()) {
            value = current.next();
            if (var.get() == value)
                break;
        }
    }

    public T selected() {
        return value;
    }

    public T next() {
        if (current.hasNext())
            value = current.next();
        return value;
    }

    public T prev() {
        if (current.hasPrevious())
            value = current.previous();
        return value;
    }

    public void apply() {
        var.set(value);
    }

    @Override
    public String toString() {
        return name;
    }
}
