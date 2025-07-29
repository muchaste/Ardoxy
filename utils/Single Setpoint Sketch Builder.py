import os
import tkinter as tk
from tkinter import filedialog, messagebox

# Function to enable/disable textboxes based on channel activation checkbox
def toggle_channel(channel_vars, check_var):
    for var in channel_vars:
        if check_var.get():
            var.config(state='normal')
        else:
            var.config(state='disabled')

# Function to check for duplicate pins
def check_for_duplicate_pins(rx_pin, tx_pin, relay_pins):
    pins_used = [rx_pin, tx_pin] + relay_pins
    duplicates = [pin for pin in pins_used if pins_used.count(pin) > 1]
    return len(duplicates) == 0

# Function to generate Arduino sketch
def generate_sketch():
    num_channels = 4  # Fixed to 4 channels
    do_setpoints = []
    kp_values = []
    ki_values = []
    kd_values = []
    relay_pins = []

    active_channels = []

    for i in range(num_channels):
        if channel_vars[i].get():  # Only process active channels
            do_setpoints.append(do_entries[i].get())
            kp_values.append(kp_entries[i].get())
            ki_values.append(ki_entries[i].get())
            kd_values.append(kd_entries[i].get())
            relay_pins.append(int(relay_entries[i].get()))  # Convert to integers
            active_channels.append(i + 1)

    if len(active_channels) == 0:
        messagebox.showerror("Input Error", "At least one channel must be activated.")
        return

    samp_interval = int(samp_interval_entry.get())
    exp_duration = int(exp_duration_entry.get())
    rx_pin = int(rx_pin_entry.get())
    tx_pin = int(tx_pin_entry.get())
    valve_closed = valve_entry.get()
    filepath = filepath_entry.get()

    # Check for duplicate pins (either relay or RX/TX)
    if not check_for_duplicate_pins(rx_pin, tx_pin, relay_pins):
        messagebox.showerror("Pin Error", "Duplicate pins detected. Each pin must be unique (RX, TX, and relay pins).")
        return

    if samp_interval < ((len(active_channels)+1)*200):
        messagebox.showerror("Sampling Time Error", f"Measurement takes {((len(active_channels)+1)*175)}ms, choose a sample interval at least twice this duration.")
        return

    # Extract filename and directory
    sketch_name = os.path.splitext(os.path.basename(filepath))[0]
    directory = os.path.dirname(filepath)
    folder_path = os.path.join(directory, sketch_name)

    # Create the folder
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Full file path including folder
    full_filepath = os.path.join(folder_path, f"{sketch_name}.ino")

    # Build the Arduino sketch template
    sketch = f"""
/*
  Auto-generated Arduino Sketch
  {len(active_channels)}-channel static DO control
*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

unsigned long sampInterval = {samp_interval};             // sampling interval in ms
unsigned int experimentDuration = {exp_duration};         // duration in min
unsigned int windowSize = round(sampInterval/200);        // PID controller will calculate an output between 0 and windowSize.
                                                          // This will be multiplied by 200 to ensure a minimum opening time of 200 msec to protect the relays.
                                                          // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec
const int channelNumber = {len(active_channels)};
const int channelArray[channelNumber] = {{{', '.join(map(str, active_channels))}}};
const int relayPins[channelNumber] = {{{', '.join(map(str, relay_pins))}}};

// Define pins
const int RX = {rx_pin};
const int TX = {tx_pin};

// DO setpoints for each channel
double DOSetpoints[channelNumber] = {{{', '.join(do_setpoints)}}};

//#######################################################################################
//###                            Requisite Variables                                  ###
//#######################################################################################

// DO measurement
double DOFloat[channelNumber];                      // Floating point DO values for each channel
long DOInt, tempInt;                                // for measurement result of each channel
double tempFloat;                                   // measurement result as floating point number
int check;                                          // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
const int closed = {valve_closed};
const int open = !closed;
const int measureDur = (channelNumber + 1) * 200;   // duration of measurement in ms (-> during this time, the system is blocked)

// Measurement timing
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement
int valveOpen[channelNumber];               // indicator if the solenoid should remain open over one loop iteration
int idcArray[channelNumber];                // array for indices of the relayPins array
int idx;

// Instances
SoftwareSerial mySer(RX, TX);
Ardoxy ardoxy(mySer);

// PID
double output[channelNumber];               // Output values from PID
"""

    # Add PID control setup and relay pins for each active channel
    for i, channel in enumerate(active_channels):
        kp = kp_values[i]
        ki = ki_values[i]
        kd = kd_values[i]
        sketch += f"""
// PID control for channel {channel}
double Kp{channel} = {kp};                  // Proportional coefficient for channel {channel}
double Ki{channel} = {ki};                  // Integral coefficient for channel {channel}
double Kd{channel} = {kd};                  // Derivative coefficient for channel {channel}
PID valvePID{channel}(&DOFloat[{i}], &output[{i}], &DOSetpoints[{i}], Kp{channel}, Ki{channel}, Kd{channel}, REVERSE);
"""

    # Add the setup function and loop
    sketch += """
//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  delay(100);
  for (int i = 0; i < channelNumber; i++){
    valveOpen[i] = 0;

    // Set up relay pin for channel {channel}
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], closed);
  }
"""

    # Add PID initialization and pin setup for each active channel
    for i, channel in enumerate(active_channels):
        sketch += f"""
  // Set up PID for channel {channel}
  valvePID{channel}.SetMode(AUTOMATIC);
  valvePID{channel}.SetSampleTime(sampInterval);
  valvePID{channel}.SetOutputLimits(0, windowSize);"""
    sketch += """
  Serial.println("------------ Auto-generated Arduino Sketch ------------");
  Serial.print("FireSting channel: ");
  Serial.println(channelNumber);
  Serial.print(" Air saturation threshold(s) (% air sat.): ");
  for (int i = 0; i < channelNumber; i++){
    Serial.print(DOSetpoints[i]);
    Serial.print("; ");
  }
  Serial.println();
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.print("Experiment Duration (min): ");
  Serial.println(experimentDuration);
  Serial.println("Use the SerialPlot software to plot DO and temperature");
  Serial.println("Send \\"1\\" to start measurement and \\"0\\" to end measurement.");
  Serial.println("-------------------------------------------------------");
}

//#######################################################################################
//###                                   Loop                                          ###
//#######################################################################################

void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          startTrigger = true;
          ardoxy.begin();                                           // Start serial communication with FireSting
          for (int i = 0; i < channelNumber; i++) {
            Serial.print("Air_sat_ch_");
            Serial.print(channelArray[i]);
            Serial.print(";");
            Serial.print("Open_time_ch_");
            Serial.print(channelArray[i]);
            Serial.print(";");
          }
          Serial.println("Temp_deg_C");
          // Define time points for decrease end and trial end
          progStart = millis();
          progEnd = progStart + experimentDuration * 60 * 1000UL;
          break;
      case '0':
          startTrigger = false;
          ardoxy.end();
          Serial.println("Stopped");
          for (int i = 0; i < channelNumber; i++) {
            digitalWrite(relayPins[i], closed);
            valveOpen[i] = 0;
          }
          break;
    }
  }

  if (startTrigger) {
    loopStart = millis();                               // get the time
    // If the end of the experiment hasn't been reached...
    if (loopStart <= progEnd){
      check = ardoxy.measureTemp();                     // measure temperature
      if(check == 1){
        tempInt = ardoxy.readoutTemp();                 // read temperature value from results register
        tempFloat = tempInt / 1000.00;
        for (int i = 0; i < channelNumber; i++) {
          check = ardoxy.measureDO(i+1);                // measure DO on channel i+1
          if(check == 1){
            DOInt = ardoxy.readoutDO(i+1);              // read DO value from results register
            DOFloat[i] = DOInt / 1000.00;               // convert to floating point number
          }
          else {                                        // If the DO measurement returns with an error
            for (int i = 0; i < channelNumber; i++) {
              digitalWrite(relayPins[i], closed);
            }
            Serial.println("Com error. Check connections and send \\"1\\" to restart.");
            ardoxy.end();
            startTrigger = false;
          }
        }
      }
      else {                                            // If the temp. measurement returns with an error
        for (int i = 0; i < channelNumber; i++) {
          digitalWrite(relayPins[i], closed);
        }
        Serial.println("Com error. Check connections and send \\"1\\" to restart.");
        ardoxy.end();
        startTrigger = false;
      }

      // compute opening time of solenoid valve"""
    for i, channel in enumerate(active_channels):
            sketch += f"""
      valvePID{channel}.Compute();"""

    sketch += """
      // Print to serial
      for (int i = 0; i < channelNumber; i++) {
        Serial.print(DOFloat[i]);
        Serial.print(";");
        Serial.print(output[i]*200/1000);
        Serial.print(";");
      }
      Serial.println(tempFloat);

      // if more than one channel: sort by opening time
      if (channelNumber > 1) {
        for (int k = 0; k < channelNumber; k++) {         // sort indices of channels in ascending order of opening time
          idcArray[k] = channelNumber-1;
          for (int m = 0; m < channelNumber; m++) {
            if (output[m] > output[k]) {
              idcArray[k] -= 1;
            }
            else if (output[m] == output[k]){
              if (m > k){
                idcArray[k] -= 1;
              }
            }
          }
        }
      }
      else {
        idcArray[0] = 0;
      }

      // operate solenoid
      // first loop: open all valves that have a nonzero PID output and close those with zero output
      for (int i = 0; i < channelNumber; i++){
        if (output[i] == 0){
          if (valveOpen[i]){
            digitalWrite(relayPins[i], closed);
            valveOpen[i] = 0;
          }
        }
        else {
          if (!valveOpen[i]) {
            digitalWrite(relayPins[i], open);
          }
          if (output[i] * 200 >= (sampInterval - measureDur)){
            valveOpen[i] = 1;
          }
          else {
            valveOpen[i] = 0;
          }
        }
      }

      // second loop: close the valves that have shorter opening times than the loop duration
      for (int i = 0; i < channelNumber; i++){
        idx = idcArray[i];
        if((!valveOpen[idx]) && (output[idx] > 0)){
          if (i == 0) {
            delay(output[idx]*200);
            digitalWrite(relayPins[idx], closed);
          }
          else {
            delay((output[idx]*200)-(output[idcArray[i-1]]*200));
            digitalWrite(relayPins[idx], closed);
          }
        }
      }

      // wait for next loop iteration
      elapsed = millis()-loopStart;
      if (sampInterval > elapsed) {
        delay(sampInterval - elapsed);
      }
    }

    // at the end of the experiment, close all valves and end the experiment
    else {
      for (int i = 0; i < channelNumber; i++) {
        digitalWrite(relayPins[i], closed);
      }
      Serial.println("End of experiment. Arduino stopps. Send \\"1\\" to re-start.");
      ardoxy.end();
      startTrigger = false;
    }
  }
}
"""

    # Write the sketch to a .ino file inside the folder
    try:
        with open(full_filepath, 'w') as f:
            f.write(sketch)
        messagebox.showinfo("Success", f"Sketch generated and saved to {full_filepath}")
    except Exception as e:
        messagebox.showerror("File Error", f"Failed to save sketch: {e}")

# Function to browse for a directory and set the file path
def browse_directory():
    filepath = filedialog.asksaveasfilename(defaultextension=".ino", filetypes=[("Arduino Sketch", "*.ino")])
    filepath_entry.delete(0, tk.END)
    filepath_entry.insert(0, filepath)

# Create the main application window
root = tk.Tk()
root.title("Single Setpoint Sketch Builder")
#root.minsize(width = 500, height = 200)

channel_vars = [tk.IntVar() for _ in range(4)]
do_entries = []
kp_entries = []
ki_entries = []
kd_entries = []
relay_entries = []

# Create the GUI layout for channels
for i in range(4):
    # Channel row with checkbox
    channel_frame = tk.Frame(root)
    channel_frame.grid(row=i, column=0, columnspan=7, pady=5)

    # Checkbox to enable/disable channel
    tk.Checkbutton(channel_frame, text=f"Channel {i+1}", variable=channel_vars[i], command=lambda i=i: toggle_channel([do_entries[i], kp_entries[i], ki_entries[i], kd_entries[i], relay_entries[i]], channel_vars[i])).grid(row=0, column=0)

    # Labels and Input fields
    tk.Label(channel_frame, text="DO Setpoint:").grid(row=0, column=1)
    do_entry = tk.Entry(channel_frame, state='disabled', width = 6)
    do_entry.grid(row=0, column=2)
    do_entries.append(do_entry)

    tk.Label(channel_frame, text="Kp:").grid(row=0, column=3)
    kp_entry = tk.Entry(channel_frame, state='disabled', width = 6)
    kp_entry.grid(row=0, column=4)
    kp_entries.append(kp_entry)

    tk.Label(channel_frame, text="Ki:").grid(row=0, column=5)
    ki_entry = tk.Entry(channel_frame, state='disabled', width = 6)
    ki_entry.grid(row=0, column=6)
    ki_entries.append(ki_entry)

    tk.Label(channel_frame, text="Kd:").grid(row=0, column=7)
    kd_entry = tk.Entry(channel_frame, state='disabled', width = 6)
    kd_entry.grid(row=0, column=8)
    kd_entries.append(kd_entry)

    tk.Label(channel_frame, text="Relay Pin:").grid(row=0, column=9)
    relay_entry = tk.Entry(channel_frame, state='disabled', width = 6)
    relay_entry.grid(row=0, column=10)
    relay_entries.append(relay_entry)

# Global Settings row for sampling interval, experiment duration, RX, TX pins, and file path
global_frame = tk.Frame(root)
global_frame.grid(row=5, column=0, columnspan=7, pady=5)

tk.Label(global_frame, text="Sampling Interval (ms):").grid(row=0, column=0)
samp_interval_entry = tk.Entry(global_frame, width = 6)
samp_interval_entry.grid(row=0, column=1)

tk.Label(global_frame, text="Experiment Duration (min):").grid(row=0, column=2)
exp_duration_entry = tk.Entry(global_frame, width = 6)
exp_duration_entry.grid(row=0, column=3)

tk.Label(global_frame, text="RX Pin:").grid(row=1, column=0)
rx_pin_entry = tk.Entry(global_frame, width = 6)
rx_pin_entry.grid(row=1, column=1)

tk.Label(global_frame, text="TX Pin:").grid(row=1, column=2)
tx_pin_entry = tk.Entry(global_frame, width = 6)
tx_pin_entry.grid(row=1, column=3)

tk.Label(global_frame, text="Valve Closed State:").grid(row=1, column=4)
valve_entry = tk.Entry(global_frame, width = 6)
valve_entry.grid(row=1, column=5)
valve_entry.insert(0, "HIGH")

tk.Label(global_frame, text="Output File Path (.ino):").grid(row=2, column=0)
filepath_entry = tk.Entry(global_frame, width = 30)
filepath_entry.grid(row=2, column=1, columnspan=4)

# Button to browse for directory
browse_button = tk.Button(global_frame, text="Browse", command=browse_directory)
browse_button.grid(row=2, column=5)

# Button to generate the sketch
generate_button = tk.Button(root, text="Generate Sketch", command=generate_sketch)
generate_button.grid(row=6, column=0, columnspan=7, pady=10)

# Start the GUI event loop
root.mainloop()
