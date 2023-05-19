

#include <Servo.h>
//Definiendo Servos
Servo servohori;
int servoh = 0;
int servohLimitHigh = 160;
int servohLimitLow = 10;

Servo servoverti;
int servov = 0;
int servovLimitHigh = 180;
int servovLimitLow = 20;
char Incoming_value = 0;
//Asignando LDRs
int ldrtopl = A2; //top left LDR
int ldrtopr = A1; //top right LDR
int ldrbotl = A3; // bottom left LDR
int ldrbotr = A0; // bottom right LDR
#include <INA219.h>

#include <Wire.h>
#include <INA219_WE.h>
#define I2C_ADDRESS 0x40
/* There are several ways to create your INA219 object:
   INA219_WE ina219 = INA219_WE()              -> uses Wire / I2C Address = 0x40
   INA219_WE ina219 = INA219_WE(ICM20948_ADDR) -> uses Wire / I2C_ADDRESS
   INA219_WE ina219 = INA219_WE(&wire2)        -> uses the TwoWire object wire2 / I2C_ADDRESS
   INA219_WE ina219 = INA219_WE(&wire2, I2C_ADDRESS) -> all together
   Successfully tested with two I2C busses on an ESP32
*/
INA219_WE ina219 = INA219_WE(I2C_ADDRESS);



void setup() {
  servohori.attach(8);
  servohori.write(0);
  servoverti.attach(9);
  servoverti.write(0);
  delay(500);
  Serial.begin(9600);
  pinMode(1, INPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  if (!ina219.init()) {
    Serial.println("INA219 not connected!");
  }
  /* Set ADC Mode for Bus and ShuntVoltage
     Mode *            * Res / Samples *       * Conversion Time
    BIT_MODE_9        9 Bit Resolution             84 µs
    BIT_MODE_10       10 Bit Resolution            148 µs
    BIT_MODE_11       11 Bit Resolution            276 µs
    BIT_MODE_12       12 Bit Resolution            532 µs  (DEFAULT)
    SAMPLE_MODE_2     Mean Value 2 samples         1.06 ms
    SAMPLE_MODE_4     Mean Value 4 samples         2.13 ms
    SAMPLE_MODE_8     Mean Value 8 samples         4.26 ms
    SAMPLE_MODE_16    Mean Value 16 samples        8.51 ms
    SAMPLE_MODE_32    Mean Value 32 samples        17.02 ms
    SAMPLE_MODE_64    Mean Value 64 samples        34.05 ms
    SAMPLE_MODE_128   Mean Value 128 samples       68.10 ms
  */
  //ina219.setADCMode(SAMPLE_MODE_128); // choose mode and uncomment for change of default

  /* Set measure mode
    POWER_DOWN - INA219 switched off
    TRIGGERED  - measurement on demand
    ADC_OFF    - Analog/Digital Converter switched off
    CONTINUOUS  - Continuous measurements (DEFAULT)
  */
  // ina219.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  /* Set PGain
    Gain *  * Shunt Voltage Range *   * Max Current (if shunt is 0.1 ohms)
    PG_40       40 mV                    0.4 A
    PG_80       80 mV                    0.8 A
    PG_160      160 mV                   1.6 A
    PG_320      320 mV                   3.2 A (DEFAULT)
  */
  // ina219.setPGain(PG_320); // choose gain and uncomment for change of default

  /* Set Bus Voltage Range
    BRNG_16   -> 16 V
    BRNG_32   -> 32 V (DEFAULT)
  */
  // ina219.setBusRange(BRNG_32); // choose range and uncomment for change of default
  // Serial.println("INA219 Current Sensor Example Sketch - Continuous");
  /* If the current values delivered by the INA219 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA219
  */
  // ina219.setCorrectionFactor(0.98); // insert your correction factor if necessary

  /* If you experience a shunt voltage offset, that means you detect a shunt voltage which is not
     zero, although the current should be zero, you can apply a correction. For this, uncomment the
     following function and apply the offset you have detected.
  */
  // ina219.setShuntVoltOffset_mV(0.5); // insert the shunt voltage (millivolts) you detect at zero current
}
void loop()   {

  {
    int i;
    float Voltage[100];
    float Current[100];
    float Power[100];
    float shuntVoltage_mV = 0.0;
    float loadVoltage_V = 0.0;
    float busVoltage_V = 0.0;
    float current_mA = 0.0;
    float power_mW = 0.0;
    bool ina219_overflow = false;

  for (i=0; i<100; i++){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();

      Serial.print("Load Voltage [V]: "); Serial.print(loadVoltage_V);
      Serial.print( ','); Serial.print('/t');
      Serial.print("Current[mA]: "); Serial.print(current_mA);

      if (!ina219_overflow) {
        Serial.println("Values OK - no overflow");
      }
      else {
        Serial.println("Overflow! Choose higher PGAIN");
      }

      Serial.println(i);

      delay(100);
    }
  }




  {
    if (Serial.available() > 0)
    { Incoming_value = Serial.read();
      Serial.print(Incoming_value);
      Serial.print("\n");

      if (Incoming_value == '1') // modo manual
      {
        if (Incoming_value == 'n') // arriba
        {
         servohori.write(servoh + 1); delay(500);
        }

        if (Incoming_value == 's') // abajo
        {
          servohori.write(servoh - 1); delay(500);
        }

        if (Incoming_value == 'e') // derecha
        {
         servoverti.write(servov + 1); delay(500);
        }

        if (Incoming_value == 'o') //  izquierda
        {
         servoverti.write(servov - 1); delay(500);
        }
      }
      if (Incoming_value == '0') // modo automatico
      {
        servoh = servohori.read();
        servov = servoverti.read();
        //capturando valores analogicos de cada LDR
        int topl = analogRead(ldrtopl);
        int topr = analogRead(ldrtopr);
        int botl = analogRead(ldrbotl);
        int botr = analogRead(ldrbotr);
        // calculando el Promedio
        int avgtop = (topl + topr) / 2; //Promedio del top LDRs
        int avgbot = (botl + botr) / 2; //Promedio del bottom LDRs
        int avgleft = (topl + botl) / 2; //Promedio del left LDRs
        int avgright = (topr + botr) / 2; //Promedio del right LDRs

        if (avgtop < avgbot)
        {
          servoverti.write(servov + 1);
          if (servov > servovLimitHigh)
          {
            servov = servovLimitHigh;
          }
          delay(100);
        }
        else if (avgbot < avgtop)
        {
          servoverti.write(servov - 1);
          if (servov < servovLimitLow)
          {
            servov = servovLimitLow;
          }
          delay(100);
        }
        else
        {
          servoverti.write(servov);
        }

        if (avgleft > avgright)
        {
          servohori.write(servoh + 1);
          if (servoh > servohLimitHigh)
          {
            servoh = servohLimitHigh;
          }
          delay(100);
        }
        else if (avgright > avgleft)
        {
          servohori.write(servoh - 1);
          if (servoh < servohLimitLow)
          {
            servoh = servohLimitLow;
          }
          delay(100);
        }
        else
        {
          servohori.write(servoh);
        }
        delay(1000);
      }

      if (Incoming_value == '2') // modo sueño
      {
        ; delay(500);
        servohori.write(0); delay(5000);
        servoverti.write(0); delay (5000);
      }





    }

  }

}
