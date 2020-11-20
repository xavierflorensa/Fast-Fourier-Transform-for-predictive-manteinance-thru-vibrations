#include "config.h"
#include "arduinoFFT.h"
#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 256 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];


TTGOClass *watch;
TFT_eSPI *tft;
BMA *sensor;
int trend[256];
int indexeixtemporal=0;
int accely;
int accelz;



void setup()
{
    Serial.begin(115200);
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
    // Get TTGOClass instance
    watch = TTGOClass::getWatch();

    // Initialize the hardware, the BMA423 sensor has been initialized internally
    watch->begin();

    // Turn on the backlight
    watch->openBL();

    //Receive objects for easy writing
    tft = watch->tft;
    sensor = watch->bma;

    // Accel parameter structure
    Acfg cfg;
    /*!
        Output data rate in Hz, Optional parameters:
            - BMA4_OUTPUT_DATA_RATE_0_78HZ
            - BMA4_OUTPUT_DATA_RATE_1_56HZ
            - BMA4_OUTPUT_DATA_RATE_3_12HZ
            - BMA4_OUTPUT_DATA_RATE_6_25HZ
            - BMA4_OUTPUT_DATA_RATE_12_5HZ
            - BMA4_OUTPUT_DATA_RATE_25HZ
            - BMA4_OUTPUT_DATA_RATE_50HZ
            - BMA4_OUTPUT_DATA_RATE_100HZ
            - BMA4_OUTPUT_DATA_RATE_200HZ
            - BMA4_OUTPUT_DATA_RATE_400HZ
            - BMA4_OUTPUT_DATA_RATE_800HZ
            - BMA4_OUTPUT_DATA_RATE_1600HZ
    */
    cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    /*!
        G-range, Optional parameters:
            - BMA4_ACCEL_RANGE_2G
            - BMA4_ACCEL_RANGE_4G
            - BMA4_ACCEL_RANGE_8G
            - BMA4_ACCEL_RANGE_16G
    */
    cfg.range = BMA4_ACCEL_RANGE_2G;
    /*!
        Bandwidth parameter, determines filter configuration, Optional parameters:
            - BMA4_ACCEL_OSR4_AVG1
            - BMA4_ACCEL_OSR2_AVG2
            - BMA4_ACCEL_NORMAL_AVG4
            - BMA4_ACCEL_CIC_AVG8
            - BMA4_ACCEL_RES_AVG16
            - BMA4_ACCEL_RES_AVG32
            - BMA4_ACCEL_RES_AVG64
            - BMA4_ACCEL_RES_AVG128
    */
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /*! Filter performance mode , Optional parameters:
        - BMA4_CIC_AVG_MODE
        - BMA4_CONTINUOUS_MODE
    */
    cfg.perf_mode = BMA4_CONTINUOUS_MODE;

    // Configure the BMA423 accelerometer
    sensor->accelConfig(cfg);

    // Enable BMA423 accelerometer
    sensor->enableAccel();

    // You can also turn it off
    // sensor->disableAccel();

    // Some display settings
    tft->setTextColor(random(0xFFFF));
    tft->drawString("BMA423 accel",  25, 50, 4);
    tft->setTextFont(4);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
}

void loop()
{
    Accel acc;
    if (indexeixtemporal==255) 
    {
      indexeixtemporal=0;
      tft->fillScreen(TFT_BLACK);
      tft->setCursor(0, 140);
      tft->setTextFont(4);
      tft->println("Calculating  FFT");
      tft->setCursor(0, 180);
      tft->println("256 samples:"); 
      
      //perform the Fourier Trnasform of the last 250 samples
      /*SAMPLING*/
      for(int i=0; i<SAMPLES; i++)
      {
        microseconds = micros();    //Overflows after around 70 minutes!
        // Get acceleration data
        vReal[i] = trend[i];
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
      }
      /*FFT*/
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
      double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
 
      /*PRINT RESULTS*/
      tft->fillScreen(TFT_BLACK);
      //Serial.println(peak);     //Print out what frequency is the most dominant.
      tft->setCursor(0, 10);
      tft->setTextFont(4);
      tft->print("Most dominant freq: "); tft->println(peak);
      for(int i=0; i<(SAMPLES/2); i++)
      {
        /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
         
        Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        Serial.print(" ");
        Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
        tft->drawLine(i,180,i,180-vReal[i]/1000,TFT_MAGENTA);
      }
 
    delay(1000);  //Repeat the process every second OR:
    //while(1);       //Run code once
    }
    // Get acceleration data
    bool res = sensor->getAccel(acc);

    if (res == false) {
        Serial.println("getAccel FAIL");
    } else {
        if (indexeixtemporal<255)
        {
          // Show the data
          tft->setTextFont(2);
          tft->fillRect(218, 120, 70, 85, TFT_BLACK);
          tft->setCursor(180, 140);
          tft->print("X:"); tft->println(acc.x);
          tft->setCursor(180, 170);
          tft->print("Y:"); tft->println(acc.y);
          tft->setCursor(180, 200);
          tft->print("Z:"); tft->println(acc.z);
          trend[indexeixtemporal]=acc.z;
          accely=abs((acc.y)/10);
          accelz=abs((acc.z)/10);
        
          if (indexeixtemporal>0)
          {
            tft->drawLine(indexeixtemporal-1,abs(trend[indexeixtemporal-1]/10),indexeixtemporal,accelz,TFT_WHITE);
          }
          indexeixtemporal=indexeixtemporal+1;
        }
    }
    //delay(100);
}
