package org.firstinspires.ftc.teamcode.FileIO;

import org.firstinspires.ftc.teamcode.RobotHardware;
import android.content.Context;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Vector;

/**
 * Created by HomeStephen on 12/12/17.
 */

public class CSV {
    private RobotHardware robotHardware;
    private Vector<Vector<Double>> dataTable = new Vector<>();
    private Vector<String> titles = new Vector<>();

    private PrintWriter writer;
    private Vector<Double> currentRecord = new Vector<>();
    private int numRecordsWritten = 0;
    private boolean isTitlesPopulated = false;
    private boolean isTitlesWritten = false;
    private boolean isOpen = false;

    public CSV(RobotHardware robotHardware)
    {
        this.robotHardware = robotHardware;
    }

    /**
     * Add a new data record to the data table.
     * Automatically attempts to write() when a records is added.
     * @param new_data_record A vector of Doubles, representing the fields a a record.
     */
    public void addRecord(Vector<Double> new_data_record)
    {
        dataTable.add(new_data_record);
        write();
    }

    public void addRecord(double[] new_data_record)
    {
        Vector<Double> vec = new Vector<>();
        for (double value : new_data_record)
            vec.add(value);
        dataTable.add(vec);
        write();
    }

    /**
     * Opens a FileOutputStream and a PrintWriter, and sets isOpen = true.
     * CSV will automatically attempt write all data cached in data_table.
     * @param fileName
     */
    public void open(String fileName) {
        try{
            FileOutputStream outputStream = robotHardware.hardwareMap.appContext.openFileOutput(fileName, Context.MODE_PRIVATE);
            writer = new PrintWriter(outputStream);
            isOpen = true;
            write();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Added field data to next record.
     * Used in conjunction with completeRecord()
     * @param title
     * @param dataField
     */
    public void addFieldToRecord(String title, Double dataField) {
        currentRecord.add(dataField);
        if(!isTitlesPopulated) {
            titles.add(title);
        }
    }

    /**
     * Adds current record to data_table, and attempts to write it.
     */
    public void completeRecord() {
        if(currentRecord.size() > 0) {
            if (dataTable.size() == 0 || currentRecord.size() == dataTable.lastElement().size()) {
                isTitlesPopulated = true;
                addRecord((Vector<Double>) currentRecord.clone());
                currentRecord.clear();
            } else { // Size error
                throw new NullPointerException("Current Record is not the same size as previous Record.");
            }
        }
    }

    /**
     * If CSV is not open, does nothing.
     * If the first record has not yet been closed (no titles populated), does nothing.
     * If titles are populated but not written, it writes the titles to file.
     * If titles are written, writes all unwritten dataRecords from dataTable to file.
     */
    private void write() {
        try{
            // Exit write() if file is not open.
            if(isOpen) {
                // Write titles once
                if(isTitlesPopulated && !isTitlesWritten) {
                    for (int i = 0; i < titles.size(); ++i) {
                        writer.print(titles.get(i));
                        if (i < titles.size() - 1)
                            writer.print(", ");
                    }
                    if (titles.size() > 0) {
                        writer.println();
                    }
                    isTitlesWritten = true; // Write once
                }

                // After Titles are written to file, begin writing records.
                if(isTitlesWritten) {
                    int n_records = dataTable.size();
                    for (int i = numRecordsWritten; i < n_records; ++i) {
                        int n_fields = dataTable.firstElement().size();
                        for (int j = 0; j < n_fields; ++j) {
                            writer.print(dataTable.get(i).get(j));
                            if (j < n_fields - 1)
                                writer.print(", ");
                        }
                        writer.println();
                        ++numRecordsWritten;
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Closes the writer file stream after attempting to write cached data.
     * @return unwrittenRecords
     */

    public int close() {
        try{
            write();
            writer.close();
            isOpen = false;
            int unwrittenRecords = dataTable.size() - numRecordsWritten;
            return unwrittenRecords;
        } catch (Exception e) {
            e.printStackTrace();
            return -1;
        }
    }



    /**
     * Depreciated. Can open, write, and close in one step.
     * Will throw error if used after calling addFieldToRecord().
     * @param fileName
     * @param titles
     */
    @Deprecated
    public void csvWriteAndClose(String fileName, Vector<String> titles)
    {
        try{
            if (!isTitlesPopulated) {
                this.titles = (Vector<String>)titles.clone();
                isTitlesPopulated = true;
            } else {
                throw new NullPointerException("Titles already populated, can't over write. " +
                        "Don't use csvWriteAndClose() and addFieldToRecord() together.");
            }
            open(fileName);
            write();
            close();
        } catch (Exception e) {
            robotHardware.telemetry.addData("Error:",e.getMessage());
            e.printStackTrace();
        }
    }
}
