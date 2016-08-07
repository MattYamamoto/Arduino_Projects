// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10     // Temperature data wire is on pin 10
#define RELAY 9             // Relay is on pin 6
#define LED 13              // On board LED pin
#define LED2 12
#define SAMPLE_FREQUENCY 2   // Sample rate per second
#define TEMP_BUFFER_SEC 30   // Number of seconds for buffer

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer = { 0x28, 0xE9, 0x1A, 0x28, 0x0, 0x0, 0x80, 0x80 };


// temperature buffer array
float tempBuffer[TEMP_BUFFER_SEC * SAMPLE_FREQUENCY];
int tempBufferSize = TEMP_BUFFER_SEC * SAMPLE_FREQUENCY;

// state variable
bool isOn;
/*
 * Setup function. Here we do the basics
 */
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0xE9, 0x1A, 0x28, 0x0, 0x0, 0x80, 0x80 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  //if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 11);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  // Relay
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(RELAY, OUTPUT);

  // Current relay is a Low Level trigger
  digitalWrite(RELAY, HIGH);
  isOn = false;


  //Intialize the temperature buffer
  for(uint8_t i = 0; i < tempBufferSize; i++)
  {
    tempBuffer[i] = -999;
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempF = sensors.getTempF(deviceAddress);
  Serial.print(" Temp F: ");
  Serial.println(tempF); // Converts tempC to Fahrenheit
}
/*
 * Main function. It will request the tempC from the sensors and display on Serial.
 */
void loop(void)
{ 
   
  sensors.requestTemperatures(); // Send the command to get temperatures

  // For debug purposes
  printTemperature(insideThermometer);

  // add the current temp to the buffer
  addToTemperatureBuffer(sensors.getTempF(insideThermometer));

  if(tempBuffer[tempBufferSize - 1] > -999)
  {
    // Check if the LED is on (indicating buffer was not full)
    // if it is, turn it off.
    if(digitalRead(LED) == HIGH){
      digitalWrite(LED, LOW);
    }
    
    if(isOn)
    {
      if(shouldTurnOff())
      {
        Serial.println("We should turn off");
        digitalWrite(RELAY, HIGH);
        isOn = false;
        digitalWrite(LED2, LOW);
      }
    }
    else
    {
      if(shouldTurnOn())
      {
        Serial.println("***************************");
        Serial.println("We should turn on");
        Serial.println("***************************");
        digitalWrite(RELAY, LOW);
        isOn = true;
        digitalWrite(LED2, HIGH);
      }
    }
  }
  else
  {
    Serial.println("Buffer not full yet!");
    digitalWrite(LED, HIGH);
  }
  
  
  delay(1/SAMPLE_FREQUENCY * 1000);
  Serial.println("");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// push temp value into buffer, shift buffer entries one.
void addToTemperatureBuffer(float temp)
{
  for(int i = tempBufferSize - 1; i >= 0; i--)
  {
      if(i == 0)
      { 
        tempBuffer[i] = temp;
        Serial.print("Setting last buffer entry to: ");
        Serial.println(tempBuffer[i]);
      }
      else
      {
        tempBuffer[i] = tempBuffer[i - 1];
      }
  }
  Serial.print("Temp is: ");
  Serial.print(temp);
  Serial.print(", [0] is: ");
  Serial.print(tempBuffer[0]);
  Serial.print(", [last] is: ");
  Serial.println(tempBuffer[tempBufferSize - 1]);
}

bool shouldTurnOn()
{
  float deltaTemp;
  int minTimePeriod = 2;
  
  if(tempBuffer[tempBufferSize - 1] == -999)
  {
    Serial.println("ShouldTurnOn(): Buffer not yet full");
    return false;
  }
  else
  {
    if(TEMP_BUFFER_SEC >= minTimePeriod)
    {
      //Change in temperature over the fist two seconds of the buffer
      deltaTemp = tempBuffer[0] - tempBuffer[SAMPLE_FREQUENCY * minTimePeriod];
      //Serial.print("first: ");
      //Serial.print(tempBuffer[0]);
     // Serial.print("second]: ");
     // Serial.println(tempBuffer[SAMPLE_FREQUENCY * minTimePeriod]);
     // Serial.print("deltaT: ");
     // Serial.print(deltaTemp);
    //  Serial.print(", and rate is :");
    //  Serial.println(deltaTemp/minTimePeriod);
      
      // Triggering on a rate of -0.3deg/sec.
      // And only need to turn on if the room temp was/is greater than 75
      if(deltaTemp/minTimePeriod < -0.3 && tempBuffer[tempBufferSize - 1] > 75)
      {
        return true;
      }
    }
    else
    {
      // Buffer is smaller than the minTimePeriod, try to use the entire
      // buffer to calculate the deltaT
      deltaTemp = tempBuffer[tempBufferSize - 1] - tempBuffer[0];

      // Triggering on a rate of 3deg/sec
      if(deltaTemp/TEMP_BUFFER_SEC < -0.2)
      {
        return true;
      }
    }
  }

  return false;
}

bool shouldTurnOff()
{
  float deltaTemp;

  // This should never be the case, but check just to be sure
  if(tempBuffer[tempBufferSize - 1] == -999)
  {
    return true;
  }
  Serial.print("turn off dT: ");
  Serial.println(tempBuffer[0] - tempBuffer[tempBufferSize - 1]);
  if( (tempBuffer[0] - tempBuffer[tempBufferSize - 1]) >= 1)
  {
    return true;
  }

  return false;
}

void printBuffer()
{
  Serial.print("[");
  for(uint8_t i = 0; i < tempBufferSize; i++)
  {
    Serial.print(tempBuffer[i]);
    Serial.print(", ");
  }
  Serial.print("]");
}

