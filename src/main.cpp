// 2023 Paulo Costa
// Periodic Interrupts for Quadrature Encoder reading and serial commands example
// It also has the PWM generation code to drive an H-bridge like the DRV8212PDSGR


#include <Arduino.h>

#include <WiFi.h>

//#include <RPi_Pico_TimerInterrupt.h>
//#include <ArduinoOTA.h>

#include <WiFiUdp.h>
#include <LittleFS.h>

#define max_wifi_str 32

char ssid[max_wifi_str];
char password[max_wifi_str];

const char* fname_wifi = "/wifi.txt";

int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer
//RPI_PICO_Timer ITimer1(1);

#define ENC1_A 2
#define ENC1_B 3

#define ENC2_A 0
#define ENC2_B 1

#define TEST_PIN 27

//#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
//#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)

//#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)

//bool timer_handler(struct repeating_timer *t)
//{
//
//}

// PWM stuff

#define MOTOR1A_PIN 16
#define MOTOR1B_PIN 17

#define MOTOR2A_PIN 14
#define MOTOR2B_PIN 15

#define SOLENOID_PIN_A 12
#define SOLENOID_PIN_B 13

int debug;
int analogWriteBits = 10; 
int analogWriteMax = (1 << analogWriteBits) - 1; 

void setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_limit = analogWriteMax * 5.5 / 7.4;
  int PWM_max = analogWriteMax;
  if (new_PWM >  PWM_limit) new_PWM =  PWM_limit;
  if (new_PWM < -PWM_limit) new_PWM = -PWM_limit;
  
  if (new_PWM == 0) {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max);
  } else if (new_PWM > 0) {
    analogWrite(pin_a, PWM_max - new_PWM);
    analogWrite(pin_b, PWM_max);
  } else {
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max + new_PWM);
  }
}

//#define HAS_VL53L0X 1
#include <Wire.h>
#ifdef HAS_VL53L0X
#include <VL53L0X.h>
VL53L0X tof;
#endif

#define HAS_INA266
#ifdef HAS_INA266
#include <INA226_WE.h>

// There are several ways to create your INA226 object:
// INA226_WE ina226 = INA226_WE(); -> uses I2C Address = 0x40 / Wire
// INA226_WE ina226 = INA226_WE(I2C_ADDRESS);   
// INA226_WE ina226 = INA226_WE(&Wire); -> uses I2C_ADDRESS = 0x40, pass any Wire Object
// INA226_WE ina226 = INA226_WE(&Wire, I2C_ADDRESS); 
 
INA226_WE ina226 = INA226_WE(0x40);
#endif

#include "robot.h"

void init_control(robot_t& robot);
void control(robot_t& robot);

#include "trajectories.h"

PID_pars_t wheel_PID_pars;


#define ADC_IN_PIN	28

#define MUXA_PIN	18
#define MUXB_PIN	19
#define MUXC_PIN	20

#define TINY_CTRL_PIN	21

static void adc_set_channel(int channel)
{
	gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | digitalPinToBitMask(MUXB_PIN) | digitalPinToBitMask(MUXC_PIN), channel << MUXA_PIN);
  //digitalWrite(MUXA_PIN, channel & 1);
  //digitalWrite(MUXB_PIN, (channel >> 1) & 1);
  //digitalWrite(MUXC_PIN, (channel >> 2) & 1);
}

uint16_t read_adc(int channel)
{
	adc_set_channel(channel); // Switch external MUX to the desired channel
	delayMicroseconds(100);
	return analogRead(A2);    // The mux connects to analog input A2
}

void readIRSensors(IRLine_t& IRLine)
{
  byte c;  // Read the five IR sensors using the AD converter
  for (c = 0; c < IRSENSORS_COUNT; c++) {
    IRLine.IR_values[(IRSENSORS_COUNT - 1) -c] = 1023 - read_adc(3 + c);
  }
}

uint32_t encodeIRSensors(void)
{
  byte c;  // Encode five IR sensors with 6 bits for each sensor
  uint32_t result = robot.IRLine.IR_values[0] >> 4; // From 10 bits to 6 bits
  for (c = 1; c < 5; c++) {
    result = (result << 6) | (robot.IRLine.IR_values[c] >> 4);
  }
  return result;
}


uint32_t interval, last_cycle;
uint32_t loop_micros;

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
  robot.dt = new_interval;   // In seconds
  wheel_PID_pars.dt = robot.dt;  
}

// Remote commands


#include "gchannels.h"
#include "file_gchannels.h"

gchannels_t udp_commands;
gchannels_t serial_commands;
commands_list_t pars_list;

const char* pars_fname = "pars.cfg";
bool load_pars_requested = false;

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("mo")) { // The 'mo'de command ...
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("u1")) { // The 'u1' command sets the voltage for motor 1
    robot.u1_req = frame.value;

  } else if (frame.command_is("u2")) { // The 'u2' command sets the voltage for motor 1
    robot.u2_req = frame.value;

  } else if (frame.command_is("w1")) { 
    robot.w1_req = frame.value;

  } else if (frame.command_is("w2")) {
    robot.w2_req = frame.value;

  } else if (frame.command_is("st")) { 
     robot.pfsm->set_new_state(frame.value);
     robot.pfsm->update_state();

  } else if (frame.command_is("dt")) { 
     set_interval(frame.value);

  } else if (frame.command_is("v")) { 
    robot.v_req = frame.value;    

  } else if (frame.command_is("w")) { 
    robot.w_req = frame.value;    

  } else if (frame.command_is("sl")) { 
    robot.solenoid_PWM = frame.value;    

  } else if (frame.command_is("xr")) { 
    robot.xe = frame.value;    

  } else if (frame.command_is("yr")) { 
    robot.ye = frame.value;    

  } else if (frame.command_is("tr")) { 
    robot.thetae = frame.value;    

  } else if (frame.command_is("xt")) { 
    traj.xt = frame.value;    

  } else if (frame.command_is("yt")) { 
    traj.yt = frame.value;    

  /*} else if (frame.command_is("kfd")) { 
    wheel_PID_pars.Kfd = frame.value;    
    serial_commands.send_command("kfd", wheel_PID_pars.Kfd);

  } else if (frame.command_is("dz")) { 
    wheel_PID_pars.dead_zone = frame.value;    
    serial_commands.send_command("dz", wheel_PID_pars.dead_zone);

  } else if (frame.command_is("kf")) { 
    wheel_PID_pars.Kf = frame.value;    
    serial_commands.send_command("kf", wheel_PID_pars.Kf);

  } else if (frame.command_is("kc")) { 
    wheel_PID_pars.Kc = frame.value;   
    serial_commands.send_command("kc", wheel_PID_pars.Kc);

  } else if (frame.command_is("ki")) { 
    wheel_PID_pars.Ki = frame.value;  
    serial_commands.send_command("ki", wheel_PID_pars.Ki);  

  } else if (frame.command_is("kd")) { 
    wheel_PID_pars.Kd = frame.value;  
    serial_commands.send_command("kd", wheel_PID_pars.Kd);  */

  } else if (frame.command_is("pl")) { 
    //load_pars();
    //load_commands(pars_fname, serial_commands);
    load_pars_requested = true;

  /*} else if (frame.command_is("pr")) { 
    serial_commands.send_command("kf", wheel_PID_pars.Kf);
    serial_commands.send_command("kc", wheel_PID_pars.Kc);
    serial_commands.send_command("ki", wheel_PID_pars.Ki);  
    serial_commands.send_command("kd", wheel_PID_pars.Kd);      
  */  
  } else if (frame.command_is("ps")) {  
    save_commands(pars_fname, pars_list, serial_commands);
   
  } else if (frame.command_is("fv")) { 
    robot.follow_v = frame.value; 

  } else if (frame.command_is("fk")) { 
    robot.follow_k = frame.value; 
  
  } else if (frame.command_is("ssid")) { 
    strncpy(ssid, frame.text, max_wifi_str - 1);
    ssid[max_wifi_str - 1] = 0;
  
  } else if (frame.command_is("pass")) { 
    if (strlen(frame.text) < 8) return;
    strncpy(password, frame.text, max_wifi_str - 1);
    password[max_wifi_str - 1] = 0;    
    
  } else if (frame.command_is("wifi")) { 
    if (frame.value == 1) {
      if (WiFi.connected()) WiFi.end();
      WiFi.begin(ssid, password); 
    } else if (frame.value == 0) {
      WiFi.end();
    }
  } else if (frame.command_is("ilambda")) {    
    robot.i_lambda = frame.value; 
  
  } // Put here more commands...
}


void send_file(const char* filename, int log_high)
{
  File f;
  f = LittleFS.open(filename, "r");
  if (!f) {
    serial_commands.send_command("err", filename);
    return;
  }

  serial_commands.flush();
  Serial.flush();

  int c;
  byte b, mask;
  if (log_high) mask = 0x80;
  else mask = 0;

  while(1) {
    c = f.read(&b, 1);
    if (c != 1) break;
    serial_commands.send_char(b | mask);
  }
  f.close();

  serial_commands.flush();
  Serial.flush();  
}

void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);

void read_PIO_encoders(void)
{
  robot.enc1 = read_PIO_encoder(0);
  robot.enc2 = read_PIO_encoder(1);
}

SerialPIO SerialTiny(SerialPIO::NOPIN, 21);

void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();  
  } 
}

const char *encToString(uint8_t enc) {
  switch (enc) {
    case ENC_TYPE_NONE: return "NONE";
    case ENC_TYPE_TKIP: return "WPA";
    case ENC_TYPE_CCMP: return "WPA2";
    case ENC_TYPE_AUTO: return "AUTO";
  }
  return "UNKN";
}


void wifi_list(void)
{
  Serial.printf("Beginning scan at %d\n", millis());
  int cnt = WiFi.scanNetworks();
  if (!cnt) {
    Serial.printf("No networks found\n");
  } else {
    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %2s %4s\n", "SSID", "ENC", "CH", "RSSI");
    for (int i = 0; i < cnt; i++) {
      uint8_t bssid[6];
      WiFi.BSSID(i, bssid);
      Serial.printf("%32s %5s %2d %4d\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
}

/*
void init_OTA(void)
{
  ArduinoOTA.setPort(2040); // this is default for RP 2040
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { 
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } 
  });  
  
  ArduinoOTA.begin();    
}*/

#ifdef HAS_INA266
int setup_ina226(void)
{
  // Set Number of measurements for shunt and bus voltage which shall be averaged
  //* Mode *     * Number of samples *
  //AVERAGE_1            1 (default)
  //AVERAGE_4            4
  //AVERAGE_16          16
  //AVERAGE_64          64
  //AVERAGE_128        128
  //AVERAGE_256        256
  //AVERAGE_512        512
  //AVERAGE_1024      1024
 
  //ina226.setAverage(AVERAGE_64); // choose mode and uncomment for change of default
  ina226.setAverage(AVERAGE_4); // choose mode and uncomment for change of default

  // Set conversion time in microseconds
  // One set of shunt and bus voltage conversion will take: 
  // number of samples to be averaged x conversion time x 2
  // 
  // * Mode *         * conversion time *
  // CONV_TIME_140          140 µs
  // CONV_TIME_204          204 µs
  // CONV_TIME_332          332 µs
  // CONV_TIME_588          588 µs
  // CONV_TIME_1100         1.1 ms (default)
  // CONV_TIME_2116       2.116 ms
  // CONV_TIME_4156       4.156 ms
  // CONV_TIME_8244       8.244 ms  
  
  //ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default
  ina226.setConversionTime(CONV_TIME_4156);
  //ina226.setConversionTime(CONV_TIME_204);
  
  // Set measure mode
  //POWER_DOWN - INA226 switched off
  //TRIGGERED  - measurement on demand
  //CONTINUOUS  - continuous measurements (default)
  
  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  
  // Set Current Range
  //* Mode *   * Max Current *
  // MA_400          400 mA
  // MA_800          800 mA (default)
  
  //ina226.setCurrentRange(MA_800); // choose gain and uncomment for change of default

  // If the current values delivered by the INA226 differ by a constant factor
  // from values obtained with calibrated equipment you can define a correction factor.
  // Correction factor = current delivered from calibrated equipment / current delivered by INA226
  
  // ina226.setCorrectionFactor(0.95);
  
  Serial.println("INA226 Current Sensor - Continuous");
  
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero  
  return 1;
}
#endif

void setup() 
{
  // Set the pins as input or output as needed
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(TEST_PIN, OUTPUT);

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
 
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  pinMode(SOLENOID_PIN_A, OUTPUT);
  pinMode(SOLENOID_PIN_B, OUTPUT);
  
  init_PIO_dual_encoders(ENC1_A, ENC2_A);

  // ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);

  analogReadResolution(10);

  analogWriteResolution(analogWriteBits);
  analogWriteFreq(16000); //16000 

  pars_list.register_command("kf", &(wheel_PID_pars.Kf));
  pars_list.register_command("kc", &(wheel_PID_pars.Kc));
  pars_list.register_command("ki", &(wheel_PID_pars.Ki));
  pars_list.register_command("kd", &(wheel_PID_pars.Kd));
  pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
  pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));

  //pars_list.register_command("ssid", ssid, max_wifi_str);
  //pars_list.register_command("pass", password, max_wifi_str);

  udp_commands.init(process_command, serial_write);
  
  serial_commands.init(process_command, serial_write);

  robot.pchannels = &serial_commands;

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  LittleFS.begin();

  float control_interval = 0.04;  // In seconds
  
  // All wheeel PID controllers share the same parameters
  wheel_PID_pars.Kf = 0.18;
  wheel_PID_pars.Kc = 0.168;
  wheel_PID_pars.Ki = 1.86;
  wheel_PID_pars.Kd = 0;
  wheel_PID_pars.Kfd = 0;
  wheel_PID_pars.dt = control_interval;
  wheel_PID_pars.dead_zone = 0.2;
  int i;
  for (i = 0; i < NUM_WHEELS; i++) {
    robot.PID[i].init_pars(&wheel_PID_pars);
  }

  // strcpy(ssid, "TP-Link_29CD");
  // strcpy(password, "49871005");
  strcpy(ssid, "Sentinel_GS");
  strcpy(password, "Sentinel");

  load_commands(pars_fname, serial_commands);

// Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);
 
  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);

  SerialTiny.begin();

  //if (ITimer1.attachInterrupt(40000, timer_handler))
  //  Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  //else
  //  Serial.println("Can't set ITimer. Select another freq. or timer");

  //init_OTA();

  //Wire.setSDA(8);
  //Wire.setSCL(9);

  Wire.begin();

  #ifdef HAS_INA266

  while (!ina226.init()) {
    Serial.println("could not connect ina226!");
    delay(100);
  }

  setup_ina226();
  #endif


  #ifdef HAS_VL53L0X

  //tof.setAddress(0x22);

  tof.setTimeout(100);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startContinuous(0);  

  #endif
  
  set_interval(control_interval);    // In seconds
  init_control(robot);
}

void loop() 
{
  if (WiFi.connected() && !ip_on) {
    // Connection established
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());
 
    // Print IP Address
    ip_on = Udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  }


  if (ip_on) {
    //ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }      
  }

  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    serial_commands.process_char(b);
    //Serial.write(b);
  }  

  if (SerialTiny.available()) { 
    b = SerialTiny.read();  // Read battery voltage and On button state from the Tiny controller
    robot.battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
    robot.button_state = b & 1;
  }

  if (load_pars_requested) {
     load_commands(pars_fname, serial_commands);
     load_pars_requested = false;
  }
  
  // Do this only every "interval" microseconds 
  uint32_t now = micros();
  uint32_t delta = now - last_cycle; 
  if (delta >= interval) {
    loop_micros = micros();
    last_cycle = now;
    //last_cycle += interval;

    // Read and process sensors
    read_PIO_encoders();
     
    robot.odometry();
    //robot.battery_voltage = 7.4; // if it could not be measured...
    readIRSensors(robot.IRLine);
    
    robot.IRLine.calcIRLineEdgeLeft();
    robot.IRLine.calcIRLineEdgeRight();
    robot.IRLine.calcCrosses();

    #ifdef HAS_VL53L0X
    if (tof.readRangeAvailable()) {
      robot.prev_tof_dist = robot.tof_dist;
      robot.tof_dist = tof.readRangeMillimeters() * 1e-3;
    }    
    #endif

    #ifdef HAS_INA266
    ina226.readAndClearFlags();
    float shuntVoltage_mV = ina226.getShuntVoltage_mV();
    float busVoltage_V = ina226.getBusVoltage_V();
    float current_mA = -ina226.getCurrent_mA();
    //float power_mW = ina226.getBusPower();
    //float loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);

    robot.i_sense = robot.i_lambda * robot.i_sense + (1 - robot.i_lambda) * current_mA * 1e-3; 
    robot.u_sense = busVoltage_V * 1000;
    #endif

    // Control the robot here by choosing:
    //   PWM_1_req and PWM_1_req  when robot.control_mode = cm_pwm
    //   v1_req and v2_req        when robot.control_mode = cm_pid
    //   v_req and w_req          when robot.control_mode = cm_kinematics
    control(robot);

    // Calc outputs
    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;
    
    robot.calcMotorsVoltage();

    robot.PWM_1 = robot.u1 / robot.battery_voltage * analogWriteMax;
    robot.PWM_2 = robot.u2 / robot.battery_voltage * analogWriteMax;

    setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

    setMotorPWM(robot.solenoid_PWM, SOLENOID_PIN_A, SOLENOID_PIN_B);

    digitalWrite(LED_BUILTIN, robot.led);
    
    // Debug information
    serial_commands.send_command("dte", delta);

    serial_commands.send_command("u1", robot.u1);
    serial_commands.send_command("u2", robot.u2);

    serial_commands.send_command("e1", robot.enc1);
    serial_commands.send_command("e2", robot.enc2);

    serial_commands.send_command("Vbat", robot.battery_voltage);

    serial_commands.send_command("vt", robot.v);

    serial_commands.send_command("xe", robot.xe);
    serial_commands.send_command("ye", robot.ye);

    serial_commands.send_command("dist", robot.dist_to_goal);
    serial_commands.send_command("ang", robot.angular_error);
    serial_commands.send_command("xt", robot.xt);
    serial_commands.send_command("yt", robot.yt);

    serial_commands.send_command("ve", robot.ve);
    serial_commands.send_command("we", robot.we);

    serial_commands.send_command("w1", robot.w1e);
    serial_commands.send_command("w2", robot.w2e);

    serial_commands.send_command("sl", robot.solenoid_PWM);

    serial_commands.send_command("is", robot.i_sense);
    serial_commands.send_command("us", robot.u_sense);

    serial_commands.send_command("mode", robot.control_mode);

    serial_commands.send_command("st", robot.pfsm->state);

    serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
    
    serial_commands.send_command("IR0", robot.IRLine.IR_values[0]);
    serial_commands.send_command("IR1", robot.IRLine.IR_values[1]);
    serial_commands.send_command("IR2", robot.IRLine.IR_values[2]);
    serial_commands.send_command("IR3", robot.IRLine.IR_values[3]);
    serial_commands.send_command("IR4", robot.IRLine.IR_values[4]);

    serial_commands.send_command("IL", robot.IRLine.IR_WaterLevel);
    serial_commands.send_command("IT", robot.IRLine.IR_tresh);

    //serial_commands.send_command("d0", robot.tof_dist);

    //serial_commands.send_command("fk", robot.follow_k);
    //serial_commands.send_command("fv", robot.follow_v);
    //serial_commands.send_command("ilambda", robot.i_lambda);
    //serial_commands.send_command("pr", robot.IRLine.pos_right);
    //serial_commands.send_command("pl", robot.IRLine.pos_left);

    serial_commands.send_command("m1", robot.PWM_1);
    serial_commands.send_command("m2", robot.PWM_2);

    serial_commands.send_command("xe", robot.xe);
    serial_commands.send_command("ye", robot.ye);
    serial_commands.send_command("te", robot.thetae);

    serial_commands.send_command("xt", traj.xt);
    serial_commands.send_command("yt", traj.yt);

    pars_list.send_sparse_commands(serial_commands);

    Serial.print(" cmd: ");
    Serial.print(serial_commands.frame.command);
    Serial.print("; ");
      
    debug = serial_commands.out_count;
    serial_commands.send_command("dbg", robot.led); 
    serial_commands.send_command("loop", micros() - loop_micros);  
     
    serial_commands.flush();   
    Serial.println();
  }
    
}

