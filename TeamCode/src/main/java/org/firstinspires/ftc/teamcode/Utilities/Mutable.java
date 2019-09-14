package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Stephen on 2/20/17.
 */

public class Mutable<T> {
    private T var;

    public Mutable(T var)
    {
        this.var = var;
    }

    public T get()
    {
        return var;
    }

    public void set(T new_value)
    {
        var = new_value;
    }

    public String toString()
    {
        return var.toString();
    }
}
