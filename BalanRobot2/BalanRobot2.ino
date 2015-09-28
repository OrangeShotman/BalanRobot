// BMA180 I2C accelerometer script for tilt sensing in Cave Pearl Project
// First working build for Raw 14 bit readings
// 2012/06/11 by Edward Mallon
// still needs code for calibrating offsets

#include <Wire.h> 
#include <math.h>
//#include <MatrixMath.h>

#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
//#define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69  
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
const int GyrXcorrection =  -11;    //Determine these by zeroing their values when sitting still
const int GyrYcorrection = -152;
const int GyrZcorrection = 0;

#define N (3)
float X[N][1]={ {1},{1},{1} };
float KAngle[N]= {0,0,0} ;



char foo;  //this is bogus code to fix a software bug in the compiler see: http://forum.arduino.cc/index.php?topic=84412.0

/* BMA180 default values:  see page 26 of datasheet
 DEF_PMODE =0, DEF_SCALE =250,  bandwidth default =20 HZ */

#define BMA180_CMD_BW_TCS  0x20 
//bits 4-7 of this register are the filtering bandwidth bits, 0-3 are the temp compensation bits (page 46)
/* the output data rate of the sensor is always set at 1200 samples/second, but this then gets filtered through the BW setting (p28)
 note that the bandwith is ~ 1/2 of these numbers in the two power save modes
 and values to load into the left most 4 bits (with <<4 shifting) of that register are: */

#define cmd_bandwidth_MASK B11110000
#define BMA180_BW_10HZ     0x00  //to filter down to 10hz, the sensor reads 253 samples, so it takes quite a while!
#define BMA180_BW_20HZ     0x01
#define BMA180_BW_40HZ     0x02
#define BMA180_BW_75HZ     0x03
#define BMA180_BW_150HZ    0x04
#define BMA180_BW_300HZ    0x05
#define BMA180_BW_600HZ    0x06
#define BMA180_BW_1200HZ   0x07
#define BMA180_BW_HIGHPASS 0x08 // high-pass: 1 Hz
#define BMA180_BW_BANDPASS 0x09 // band-pass: 0.2 Hz ... 300 Hz 
// For slow moving tilt sensing applications, a bandwidth of 50Hz will probably be ok

#define BMA180_RANGEnSMP 0X35  //7.7.1 address of combined Offx, range & smp_skp register (53 decimal) - 2G default sensitivity
//and the three sensitivity range bits to put into that register are:
#define range_MASK          B00001110
#define BMA180_RANGE_1G     0x00
#define BMA180_RANGE_1DOT5G 0x01
#define BMA180_RANGE_2G     0x02
#define BMA180_RANGE_3G     0x03
#define BMA180_RANGE_4G     0x04
#define BMA180_RANGE_8G     0x05
#define BMA180_RANGE_16G    0x06

#define smp_skip_MASK       B00000001

/* 4 standard Chip power modes see p.28 of the datasheet */
// there is also sleep mode, and self "Wake up" mode where it sleeps for a while and wakes up see 7.7.3

#define MODE_config_MASK       B00000011 //section 7.7.3 (only uses 2 bits of this register for configureing the power mode)
/* AFTER write of mode_config bits in EEPROM, YOU MUST DO a soft RESET*/
#define MODE_LOW_NOISE         B00000000  // 0x00 //this is the default value!
/* low noise mode= Highest current draw, low noise, full bandwith (1200Hz) 
 page 29: "The sensor is calibrated for mode_config = '00'. By changing to other modes, the offset is changed too, thus subsequently an offset correction has to be performed. */
#define MODE_ULTRA_LOW_NOISE   B00000001  // 0x01 
/* Ultra low noise mode - highest current, lowest noise, reduced bandwith (300Hz) */
#define MODE_LPWR_LOW_NOISE    B00000010  // 0x02 
/* Low noise mode with reduced power: real bw = 1/2 set bandwith, output datarate 1200 -  */
#define MODE_LOW_POWER         B00000011  // 0x03  
/* low power mode: real bw = 1/2 set bandwith, lowest power, 
 noise higher than in low noise modes, output data rate = 1200 samples/sec */
/*  MODE  -- for 1, 1.5 and 2g
 0x00=LowNoise,       maxBW=1200 Noise 150 ug/rt
 0x01 Ultra Low Noise maxBW=472  Noise 150 ug/rt 
 0x02 Low Noise Low Power maxBW= 236  Samp/Sec =1200 Noise 200 ug/rt
 0X03 Low Power       maxBW=600                      Noise 200 ug/rt
 see Page 28 section 7.7.3 for full info */

/* the set of Data Register addresses */
#define BMA180_CMD_CHIP_ID          0x00
#define BMA180_CMD_VERSION          0x01
#define BMA180_CMD_ACC_X_LSB        0x02  /* First of 6 registers of accel data */
#define BMA180_CMD_ACC_X_MSB        0x03
#define BMA180_CMD_ACC_Y_LSB        0x04
#define BMA180_CMD_ACC_Y_MSB        0x05
#define BMA180_CMD_ACC_Z_LSB        0x06
#define BMA180_CMD_ACC_Z_MSB        0x07
#define BMA180_CMD_TEMP             0x08

/* the device status registers - info about alert/interrupt & if data is availiable to read out*/
#define BMA180_CMD_STATUS_REG1      0x09  //1st tap, alert, slope signs, offset, ee_write
#define BMA180_CMD_STATUS_REG2      0x0A  //tap and slope sense bits
#define BMA180_CMD_STATUS_REG3      0x0B  //interrupt statuses from the tap sensing
#define BMA180_CMD_STATUS_REG4      0x0C  //high signs, x&y&z tapsense

/* Control Register set: these setup functional modes */
#define BMA180_CMD_RESET            0x10 
/* reset register: set to 0 for soft reset -all register values will reset*/
/* EEprom default register values are copied to the "volatile" registers after 
 power on or soft reset. after every write command EEprom has to be reset by soft-reset.
 No serial transaction should occur within minimum 10 us after soft_reset command */

#define BMA180_CMD_CTRL_REG0        0x0D // EE_W enable write control is bit 4 of this register address!
#define BMA180_CMD_CTRL_REG1        0x0E // contains the offsets for x, y,z
#define BMA180_CMD_CTRL_REG2        0x0F // unlocking eeprom register
#define BMA180_CMD_CTRL_REG3        0x21 // configure different interrupt trigger bits in this register
#define BMA180_CMD_CTRL_REG4        0x22 // low_hy, mot_cd_r, ff_cd_tr, offset_finetuning

/* "permanent" EEprom writing is an indirect procedure. Data from corresponding volatile image registers are written 
 to EEPROM after sending write transaction to addresses 40h to 5FH (ie: above all of our "normal" register traffic) 
 The eeprom is only rated to 1000 write cycles so don't do this too often. NOTE ee_w is mis-named as it really 
 allows writing to the "volatile" image registers, and does not necessarily affect the eeprom data unless you write to 40h or above */

/* CTRL_REGISTER 0 BIT MASKS - the really important ones! see page 26 for defaults*/
#define ctrl_reg0_dis_wake_up_MASK  B00000001  /* set to 1 and unit sleeps automatically for wake_up_dur (7.7.9) then takes readings, 
set this register bit to 0 to disable the automatic sleep&wake-up mode */ 
#define ctrl_reg0_sleep_MASK        B00000010  /* chip will sleep if this bit set to 1 and wake when set to 0 
Sleep bit should not be set to "1", when wake up mask is set to "1",  wait 10ms before any eeprom operation on wake from sleep*/
#define ctrl_reg0_ee_w_MASK         B00010000  /* set this to 1 to Unlock writing to addr from 0x20 to 0x50, (default)=0 on reset
7.10.3 ee_w  This bit must be written 1 to be able to write anything into image registers, resetting ee_w to 0 will prevent any register updates*/

/* CTRL_REG0 BIT MASKS - the ones I am not using */
#define ctrl_reg0_st0_MASK          B00000100  /* self-test 0 (electrostatic) setting this bit to 1 starts the test */
#define ctrl_reg0_st1_MASK          B00001000  /* self-test 1 (digital only) setting this bit to 1 starts the test*/
#define ctrl_reg0_update_image_MASK B00100000  /* EEprom default data are downloaded to image registers when this control bit set to 1 (like a soft reset!) */
#define ctrl_reg0_reset_int_MASK    B01000000  /* if interrupts are latched, you need to write "1" to this bit to clear the latched interrupt */

/* BMA180_CTRL_REG3 MASK */
#define ctrl_reg3_new_data_int_MASK  B00000010  /* BIT(1) Set to 1, for Intrupt to occur when new accel data is ready in all three channels */

/* as a starting point see BMA180 triple axis accelerometer sample code
 from http://www.geeetech.com/wiki/index.php/BMA180_Triple_Axis_Accelerometer_Breakout */

#define BMA180 0x40  //address of the accelerometer with SDO pulled up to VCC (0x41 if SDO pulled down to GND)

#define filterSamples   7             // #samples for filtering should  be an odd number, no smaller than 3 - works great at 13 & good at 7or9 samples
int sensSmoothBMAx [filterSamples];   // array for holding raw sensor values for x 
int sensSmoothBMAy [filterSamples];   // array for holding raw sensor values for y 
int sensSmoothBMAz [filterSamples];   // array for holding raw sensor values for z 

float smoothBMAx;  // smoothed x data
float smoothBMAy;  // smoothed y data
float smoothBMAz;  // smoothed z data

float pitch,roll,result,x2, y2, z2;

//byte BMAtemperature; //not used here but availiable

// 3 color LED pin connections
#define RED_PIN 3
#define BLUE_PIN 5
#define GREEN_PIN 4




void setup() 
{ 
  Wire.begin();
  Serial.begin(9600);   
  AccelerometerInit(); 
  initGyro();
  //Serial.println("...BMA180 has been initialized");
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(3,OUTPUT);

} 


void AccelerometerInit() {  

  Serial.print("BMA180 Reset ");
  i2c_writeReg(BMA180,BMA180_CMD_RESET,0xB6);  //B6 (hex) forces the reset - pretty sure this can be done before the ee bit is set
  delay(10); //delay serial comms after reset  
  //Control, status & image registers are reset to values stored in the EEprom. 
  //puts the BMA in wake-up mode & default low noise mode "00", BW=1200
  Serial.print("Getting chip ID... ");  
  int id = i2c_readReg(BMA180, BMA180_CMD_CHIP_ID);
  if(id == 0x03)
  {
    Serial.print("BMA180 Chip found at: "); 
    Serial.print(id); 
    if (i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG0,1, ctrl_reg0_ee_w_MASK) == 0) //if you can enable register writing...
    {
      Serial.print("BMA180 Write Init Pass");

      // disable wakeup mode because we will be sleeping the sensor manually
      i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_dis_wake_up_MASK);  
      Serial.print("Wake up mode disabled, ");

      // Connect to the bw_tcs register and set the BW filtering level to 10Hz, Only bits 4-7 of the register hold this data
      i2c_writeRegBits(BMA180,BMA180_CMD_BW_TCS,BMA180_BW_40HZ,cmd_bandwidth_MASK);
      Serial.print("Filtering level set to 40HZ, ");

      // Connect to the offset_lsb1 register and set the range
      i2c_writeRegBits(BMA180,BMA180_RANGEnSMP,BMA180_RANGE_1G,range_MASK);
      Serial.print("Range set to 1G");

      /* since this is a tilt sensing application, I am using the 1g range, which is factory calibrated
       To enable the factory calibrated offset registers to be used by the sensors DAC
       en_offset_x, en_offset_y, en_offset_z control bits are set to 1 
       p49: to regulate all axis, it is necessary to enable the en_offset bits sequentially */

      i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG1,0,B10000000); //en_offset_x  not optional writing here!
      i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG1,0,B01000000); //en_offset_y
      i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG1,0,B00100000); //en_offset_z

      // some people mention sample skipping (see p29) can reduce noise?
      // usually use this with new_data_int=1 which occurs at the end of the Z-axis output register update
      // in low power mode, bw-5hz and interrupt is generated at 10hz with smpl skip on
      // i2c_writeOptionallyTo(BMA180,BMA180_RANGEnSMP,1,smp_skip_MASK);
      // Serial.print("Sample Skipping turned on");

      //  BMAtemperature = i2c_readReg(BMA180, BMA180_CMD_TEMP);
      //  Serial.print("Temperature =  ");Serial.println(BMAtemperature);

      i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_ee_w_MASK);//final step in setup is to disable register writing

    }
    else
    {  
      Serial.print("BMA180 Write Init Fail");       
    }
  }
  else
  {
    Serial.print("BMA180 Chip Detect Fail");     
  }

  AccelerometerRead();  //just to get the sensor arrays loaded
}
//
unsigned int UpdateCount = 0;
unsigned long UpdateTime = 0;
float GyroX = 0;
float GyroY = 0;
float PreviousAngle = 0;
float pidP = 0;//99;
float pidI = 0;//16
float pidD = 800;//120
float StaticError = 0;
float OutputValue = 0;
unsigned char forwardDirection = 0;
unsigned char backDirection = 0;
void loop() 
{   
  unsigned long TimeDelay = millis();
  UpdateCount++;
  AccelerometerRead(); 
  int gyro[4];
  getGyroscopeData(gyro); 
  GyroX += gyro[0]*0.0017;
  GyroY -= gyro[1]*0.0017;
  
// Work out the squares
  x2 = smoothBMAx * smoothBMAx; // getXValFloat() gets the measured g-force in m/ss
  y2 = smoothBMAy * smoothBMAy;
  z2 = smoothBMAz* smoothBMAz;
//X Axis
  pitch = atan(smoothBMAx/sqrt(y2+z2)) * 360 / PI / 2;
//Y Axis
  roll = atan(smoothBMAy/sqrt(x2+z2)) * 360 / PI / 2;
  GyroY = (GyroY*1)+(pitch*0);
  //Serial.print("Data|");Serial.print(pitch);Serial.print("|");Serial.println(GyroY); 
  StaticError = constrain(GyroY +StaticError , -255, 255);
  OutputValue = GyroY * pidP + (GyroY - PreviousAngle)*pidD + StaticError * pidI;
  OutputValue = constrain(OutputValue,-255,255);
  if (Serial.available() > 0) {  //если есть доступные данные
        // считываем байт
        byte incomingByte = Serial.read();
        if(incomingByte == 113){
          pidP++;
        }
        if(incomingByte == 119){
          pidI++;
        }
        if(incomingByte == 101){
          pidD++;
        }
        if(incomingByte == 97){
          pidP--;
        }
        if(incomingByte == 115){
          pidI--;
        }
        if(incomingByte == 100){
          pidD--;
        }
        // отсылаем то, что получили
        Serial.print("P = ");
        Serial.print(pidP);
        Serial.print("I = ");
        Serial.print(pidI);
        Serial.print("D = ");
        Serial.println(pidD);
    }
  if(OutputValue>=0){
    analogWrite(9,OutputValue);
    analogWrite(3,OutputValue);    
    analogWrite(10,0);
    analogWrite(11,0);
    //Serial.print("Direction is Forward "); Serial.println(OutputValue);
  }else{
    analogWrite(10,-OutputValue);
    analogWrite(11,-OutputValue);
    analogWrite(9,0);
    analogWrite(3,0);
    //Serial.print("Direction is Back "); Serial.println(-OutputValue);
  }
  if(millis() - UpdateTime > 1000){
    UpdateTime = millis();
    //Serial.print("AccPitch = ");Serial.print(pitch);Serial.print(" GyroY = "); Serial.print(GyroY); Serial.print("AccRoll = ");Serial.print(roll);Serial.print(" GyroX = ");Serial.print(GyroX);
    
    //Serial.print("  ");
    //Serial.print(UpdateCount);
    //Serial.print("Hz");
    //Serial.println();
    UpdateCount = 0;
  } 
  while(millis() - TimeDelay <25){
    
  }
  PreviousAngle = GyroY;
  //delay(10);
  // now put BMA189 TO SLEEP
  //i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG0,1, ctrl_reg0_sleep_MASK); //sleeps sensor if this bit set to 1
  //Serial.print("         Sleep for 1.5 sec..");   
  //delay(1);  
  //delay(10); // sleep for a while 
  //i2c_writeRegBits(BMA180,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_sleep_MASK);//and wakes when set to 0   
  //delay(1);
  //delay(10); // give some time for bma180 axis registers to fill again after waking
}

//---------------- Functions--------------------

void AccelerometerRead() 
{ 
  // read in the 3 axis data, each one is 14 bits = +- 16,383 for integer values
  // note negative values in the directions of the arrows printed on the breakout board!

  byte temp =0;
  int data = 0;
  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays

    //note negative values in the directions of the arrows printed on my breakout board!

    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_X_MSB);
    data = temp << 8; // puts the most sig bits on the corect side - I am reading 14 bits total
    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_X_LSB); 
    data |= temp >> 2; //this shift gets rid of two non-value bits in LSB register
    sensSmoothBMAx[thisReading]=data;
    data = 0;

    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_Y_MSB);
    data = temp << 8;
    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_Y_LSB);
    data |= temp >> 2; // what about adding the offset here?
    sensSmoothBMAy[thisReading]=data;
    data = 0;

    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_Z_MSB);
    data = temp << 8;
    temp = i2c_readReg(BMA180, BMA180_CMD_ACC_Z_LSB);
    data |= temp >> 2;
    sensSmoothBMAz[thisReading]=data;
    data = 0; 

    //delay(110); // we have the internal BMA bandwith filter set to 10 Hz so its not going to give new data without some time!
  }
  // Now send those readings out to the digital smoothing function
  smoothBMAx = digitalSmooth(sensSmoothBMAx);
  smoothBMAy = digitalSmooth(sensSmoothBMAy);
  smoothBMAz = digitalSmooth(sensSmoothBMAz);
}


// this smoothing function based on Paul Badger's  http://playground.arduino.cc/Main/DigitalSmooth
// "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
int digitalSmooth(int *sensSmoothArray){     
  int j, k, temp, top, bottom;
  long total;
  static int i;
  boolean done;

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sensSmoothArray[j] > sensSmoothArray[j + 1]){     // numbers are out of order - swap
        temp = sensSmoothArray[j + 1];
        sensSmoothArray [j+1] =  sensSmoothArray[j] ;
        sensSmoothArray [j] = temp;
        done = 0;
      }
    }
  }

  /*  Serial.println();
   for (j = 0; j < (filterSamples); j++){    // print the array for debugging
   Serial.print(sensSmoothArray[j]); 
   Serial.print("   "); 
   }
   Serial.println();
   */

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sensSmoothArray[j];  // total remaining indices
    k++; 
    // Serial.print(sensSmoothArray[j]);Serial.print("   "); //more debugging
  }
  //  Serial.println(); Serial.print("average = "); Serial.println(total/k); //more debugging
  return total/k;    // divide by number of samples

}

/* writeOptionallyTo based on  http://www.centralnexus.com/seismograph/details_bma180.html
 Writes val to address register on device only if it's different from the current value, using the bitmask
 Use this function of you write to the "permanent" eeprom (40 or above), as it has a limitied 1000 write lifespan
 
 byte i2c_writeOptionallyTo(int DEVICE, byte address, byte val, byte mask) {
 byte result;  
 byte value = 0;
 value = i2c_readReg(DEVICE, address);
 if ((value & mask) != (val & mask)) {  //if read in bit (selected by the mask) is "Not equal to" passed in val for the bit 
 //Keep the unmasked values, and changed the masked value to the new one 
 result = i2c_writeReg(DEVICE, address, (value & ~mask) | (val & mask));    //the squiggle is a bitwise NOT operator and pipe is bitwise OR
 }
 return result;
 }  
 */

/* Writes val to address register bits on device using the bitmask */
byte i2c_writeRegBits(int DEVICE, byte address, byte val, byte mask) {
  byte result;  
  byte value = 0;
  value = i2c_readReg(DEVICE, address);
  result = i2c_writeReg(DEVICE, address, (value & ~mask) | (val & mask));  
  return result;
}

// ReadReg & WriteReg FUNCTIONS  
// based on https://github.com/makerbot/BMA180-Datalogger/blob/master/bma180-datalogger-shield/bma180-logger/bma180.ino

byte i2c_readReg(int dev_i2c_address, byte reg_address)  //MUST be interger for the i2c address
{
  byte temp;

  Wire.beginTransmission(dev_i2c_address);
  Wire.write(reg_address);
  Wire.endTransmission();        //end transmission

    Wire.beginTransmission(dev_i2c_address); //start transmission to ACC
  Wire.requestFrom(dev_i2c_address, 1);
  while(Wire.available())
  {
    temp = Wire.read();
  }

  return temp;
}

byte i2c_writeReg(int dev_i2c_address, byte reg_address, byte data)  // used in the i2c_writeOptionallyTo function
{
  byte result;

  Wire.beginTransmission(dev_i2c_address);
  Wire.write(reg_address);
  Wire.write(data);
  result = Wire.endTransmission();

  //do some error checking
  if(result > 0)
  {
    Serial.print("PROBLEM..... Result code is ");
    Serial.println(result); 
    digitalWrite(RED_PIN, HIGH);
  }
  else
  {
    //Serial.println(" ");
  }
  delay(5);  //the BMA180 often needs some settling time after register writing

  return result;
} 

void initGyro()
{
 writeTo(GYRO, G_PWR_MGM, 0x00);
 writeTo(GYRO, G_SMPLRT_DIV, 0x04); // Internal Sample Rate/(04h +1) = 1kHz/5 = 5mS
 writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz Internal Sample,  5Hz LPF ,  0001 1101
 writeTo(GYRO, G_INT_CFG, 0x00);
}
void getGyroscopeData(int * result)
{
 int regAddress = 0x1B;
 int temp, x, y, z;
 byte buff[8];
 readFrom(GYRO, regAddress, 8, buff); //read the gyro data from the ITG3200
 result[3] = ((buff[0] << 8) | buff[1]); // temperature
 result[0] = ((buff[2] << 8) | buff[3]) - result[3]/500 + GyrXcorrection;
 result[1] = ((buff[4] << 8) | buff[5]) - result[3]/90  + GyrYcorrection;
 result[2] = ((buff[6] << 8) | buff[7]) + GyrZcorrection;
 }


//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on ACC in to buff array
 void readFrom(int DEVICE, byte address, int num, byte buff[]) {
 Wire.beginTransmission(DEVICE); //start transmission to ACC 
 Wire.write(address);        //sends address to read from
 Wire.endTransmission(); //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
 
 int i = 0;
 while(Wire.available())    //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read(); // receive a byte
   i++;
 }
 Wire.endTransmission(); //end transmission
}
