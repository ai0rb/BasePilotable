// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.playback;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
/**
 * Class for writing a list of values (in JSON) to a file
 *
 *
 *
 */
public class Recorder {
    private BufferedWriter bw;
    private File file;
    private JSONArray data;
    private JSONObject completeObject;
    private long initialTime;
    private Object[] previousValues, defaults;
    private String fileURL, recordingID;
    private String[] keys;
    /**
     * Constructor
     *  This method uses the default file of "recording.json"
     * and recording name of "recording".
     *
     * @param keys
     *            Keys to use for data
     * @param defaults
     *            Defaults for the keys. Must have the same number of elements
     *            as {@code keys}
     * @throws IllegalArgumentException
     *             If {@code keys.length} is not identical to
     *             {@code defaults.length}
     */
    public Recorder(String[] keys, Object[] defaults) throws IllegalArgumentException {
        if (keys.length != defaults.length) {
            throw new IllegalArgumentException();
        }
        recordingID = "recording";
        this.keys = keys;
        this.defaults = defaults;
        initialize("recording.json");
        //System.out.println("recording");
    }
    /**
     *
     *
     * @param keys
     *            Keys to use for data@param defaults Defaults for the keys.
     *            Must have the same number of elements as {@code keys}
     * @param fileURL
     *            Location of the file in which to write the values
     * @throws IllegalArgumentException
     *             If {@code keys.length} is not identical to
     *             {@code defaults.length}
     */
    public Recorder(String[] keys, Object[] defaults, String fileURL) {
        if (keys.length != defaults.length) {
            throw new IllegalArgumentException();
        }
        recordingID = "recording";
        this.keys = keys;
        this.defaults = defaults;
        initialize(fileURL);
    }
    /**
     *
     * @param keys
     *            Keys to use for data@param defaults Defaults for the keys.
     *            Must have the same number of elements as {@code keys}
     * @param fileURL
     *            Location of the file in which to write the values
     * @param recordingID
     *            What to call the recording in the JSON object
     * @throws IllegalArgumentException
     *             If {@code keys.length} is not identical to
     *             {@code defaults.length}
     */
    public Recorder(String keys[], Object[] defaults, String fileURL, String recordingID) {
        if (keys.length != defaults.length) {
            throw new IllegalArgumentException();
        }
        this.recordingID = recordingID;
        this.keys = keys;
        this.defaults = defaults;
        initialize(fileURL);
    }
    private boolean initialize(String fileURL) {
        completeObject = new JSONObject();
        data = new JSONArray();
        initialTime = System.currentTimeMillis();
        previousValues = new Object[keys.length];
        for (int i = 0; i < keys.length; i++) {
            previousValues[i] = defaults[i];
        }
        appendData(defaults, initialTime);
        return openFile(fileURL);
    }
    /**
     * Adds a global attribute to the recording
     *
     * @param key
     *            Key to use
     * @param data
     *            What to associate with the key
     */
    @SuppressWarnings("unchecked")
    public void addAttribute(String key, String data) {
        completeObject.put(key, data);
    }
    /**
     * Adds the essentials to the json object
     *
     * @return The finalized json object
     */
    @SuppressWarnings("unchecked")
    private JSONObject makeFinalJSON() {
        JSONObject recording = new JSONObject();
        recording.put("id", recordingID);
        recording.put("time", (new Date(initialTime)).toString());
        recording.put("data", data);
        completeObject.put("recording", recording);
        return completeObject;
    }
    /**
     * Opens the file at {@code fileURL}. If a file already exists at that
     * location, a number in parentheses is appended to the end of the filename.
     *
     * @param fileURL
     *            Which file to open
     */
    private boolean openFile(String fileURL) {
        try {
            // See if the BufferedWriter is open. If so, finish what it's doing.
            if (bw != null) {
                bw.close();
            }
            int i = 1;
            file = new File(fileURL);
            // Figures out what the file name will be
            while (file.exists()) {
                int index = fileURL.lastIndexOf('.');
                if (index != -1 && index != fileURL.length() - 1) {
                    file = new File(fileURL.substring(0, index) + "(" + i + ")" + fileURL.substring(index));
                } else {
                    file = new File(fileURL + " (" + i + ")");
                }
                i++;
            }
            this.fileURL = fileURL;
            bw = new BufferedWriter(new FileWriter(file));
            return true;
        } catch (IOException ioe) {
            bw = null;
            ioe.printStackTrace();
            return false;
        }
    }
    /**
     * Returns location of file
     *
     * @return fileURL
     */
    public String getFileURL() {
        return fileURL;
    }
    /**
     * Adds data to the JSON array. Don't forget to close the Recorder to
     * actually save the data!
     *
     * @param values
     *            Name of values to store to keys (as initialized by
     *            constructor)
     * @throws IllegalArgumentException
     *             If the number of keys does not match that of values.
     */
    public void appendData(Object[] values) throws IllegalArgumentException {
        if (keys.length != values.length) {
            throw new IllegalArgumentException("keys.legnth must equal values.length");
        }
        appendData(values, System.currentTimeMillis());
    }
    /**
     * Adds data to the JSON array. This method allows programmers to specify
     * the absolute time in milliseconds at which the data is added.
     *
     * @param values
     *            Name of values to store to keys (as initialized by
     *            constructor)
     * @param time
     *            When the recording happened
     */
    @SuppressWarnings("unchecked")
    private void appendData(Object[] values, long time) {
        JSONObject toAdd;
        boolean changed = false;
        toAdd = new JSONObject();
        // Tell when the action was recorded (relative to the initial time)
        toAdd.put("time", time - initialTime);
        for (int i = 0; i < keys.length; i++) {
            toAdd.put(keys[i], values[i]);
            // Check to see if the values have changed at all since the previous
            // iteration
            if (previousValues[i] == null || !values[i].equals(previousValues[i])) {
                changed = true;
            }
        }
        // Write to the array iff the values have been changed (saves space!)
        if (changed) {
            data.add(toAdd);
        }
        previousValues = values;
    }
    /**
     * Must be called at the end of use
     *
     * @param readable
     *            Whether or not to make the file more easily readable. Used
     *            mostly for testing purposes.
     */
    public void close(boolean readable) {
        appendData(defaults);
        try {
            // Actually writes the data to the file
            if (readable) {
                bw.write(quickFormat(makeFinalJSON().toString()));
            } else {
                bw.write(makeFinalJSON().toString());
            }
            bw.newLine();
            bw.close();
            bw = null;
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
    }
    /**
     * Must be called at the end of use
     */
    public void close() {
        close(false);
    }
    /**
     * Makes sure that the data is written to the file.
     */
    public void finalize() {
        if (bw != null) {
            close();
        }
    }
    /**
     * Makes the JSON more readable (assumes that it has good syntax). This is
     * optional to use.
     *
     * @param input
     *            String of JSON
     * @return Formatted String of JSON
     */
    private String quickFormat(String input) {
        int indent = 0;
        String formatted = "";
        // For every character in the input string
        for (int i = 0; i < input.length(); i++) {
            // Check what kind of character it is
            if (input.charAt(i) == '{' || input.charAt(i) == '[') {
                // For open braces/brackets, increase the indent, and insert a
                // new line
                indent++;
                formatted += input.charAt(i) + "\n" + (new String(new char[indent]).replace("\0", "\t"));
            } else if (input.charAt(i) == '}' || input.charAt(i) == ']') {
                // For closing braces/brackets, decrease the indent, and insert
                // a new line
                indent = Math.max(indent - 1, 0);
                formatted += "\n" + (new String(new char[indent]).replace("\0", "\t")) + input.charAt(i);
            } else if (input.charAt(i) == ',') {
                // For commas, insert a new line
                formatted += ",\n" + (new String(new char[indent]).replace("\0", "\t"));
            } else {
                // Otherwise, just add the character to the formatted string
                formatted += input.charAt(i);
            }
        }
        return formatted;
    }
}