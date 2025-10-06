# Corto-Densimeter

DIY densimeter for Corto Brewery use cases.

# ===[ Espressif IDF - ESP32-C6 ]==================================

---

## **Prerequisites**
- **Operating System**: Ubuntu 24 (or any Linux-based system).
- **ESP-IDF Version**: v5.5.1.
- **Python**: Python 3.6 or later.
- **Hardware**: ESP32-C6-DevKitC-1 board.

---

## **Setup Instructions**

### **1. Install ESP-IDF**
If ESP-IDF is not already installed, follow the [official installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html). Use the VSCode extension. 

### **2. Set Up Permissions**
To allow your system to communicate with the ESP32-C6 board, you need to set up **udev rules** :
```bash
sudo cp --update=none ~/.espressif/tools/openocd-esp32/v0.12.0-esp32-20250707/openocd-esp32/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
```
After copying the rules, reload `udev`:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### **3. Add ESP-IDF to Your Environment**
Ensure the `idf.py` command is available in your terminal. If not, activate the ESP-IDF environment:
```bash
. \$HOME/esp/v5.5.1/esp-idf/export.sh
```
To avoid running this command every time you open a terminal, you can add it to your `.bashrc`

## **Project Commands**

### **Configure the Target Board**
Set the target to **ESP32-C6** :
```bash
idf.py set-target esp32c6
```

### **Build the Project**
Compile the project :
```bash
idf.py build
```

### **Find the ESP32-C6 Port**
List available serial ports to identify the ESP32-C6 :
```bash
ls /dev/tty*
```
The ESP32-C6 often appears as `/dev/ttyACM0` or `/dev/ttyUSB0`.

### **Flash the Project**
Upload the compiled binary to the board :
```bash
idf.py -p /dev/ttyACM0 flash
```

### **Monitor Serial Output**
View the serial output of the board :
```bash
idf.py -p /dev/ttyACM0 monitor
```


