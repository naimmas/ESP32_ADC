#include <Arduino.h>
#include "esp_adc_cal.h"
#include "driver/gpio.h"

#define AN_Pot1     GPIO_NUM_4
#define AN_Pot2     GPIO_NUM_2
#define AN_Pot3     GPIO_NUM_15
#define AN_Pot4     GPIO_NUM_0
 #define FILTER_LEN  15

uint32_t AN_Pot1_Buffer[4][FILTER_LEN] = {{ 0 }, { 0 }, { 0 }, { 0 }};
int AN_Pot1_i = 0;
int AN_Pot_Raw[4] = { 0 };
int AN_Pot1_Filtered = 0;
float AN_Pot_Fil[4] = { 0.0 };

static esp_adc_cal_characteristics_t *adc_chars = NULL;
 
#define DEFAULT_VREF    1053        //Use adc2_vref_to_gpio() to obtain a better estimate
static const adc_unit_t unit = ADC_UNIT_2;
 

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

float readADC_Avg(float ADC_Raw)
{
  int i = 0;
  float Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/(float)FILTER_LEN);
}

LowPass<2> lp1(5,1e3,true);
LowPass<2> lp2(5,1e3,true);
LowPass<2> lp3(5,1e3,true);
LowPass<2> lp4(5,1e3,true);

void setup() {
  
  Serial.begin(115200);
  Serial.println("Hello world");
  
  // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
  //   printf("eFuse Vref: Supported\n");
  //   adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  //   for(int atten = ADC_ATTEN_DB_0; atten <= ADC_ATTEN_DB_11; atten++) {
  //     for(int width = ADC_WIDTH_BIT_9; width <= ADC_WIDTH_BIT_12; width++) {
  //   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, (adc_atten_t)atten, (adc_bits_width_t)width, DEFAULT_VREF, adc_chars);
  //   if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
  //     printf("atten=%d,bit_width=%d,coeff_a=%d,coeff_b=%d,vref=%d\n", adc_chars->atten, adc_chars->bit_width, adc_chars->coeff_a, adc_chars->coeff_b, adc_chars->vref);
  //   } else {
  //     printf("N/A\n");
  //   }
  //     }
  //   }
  // }
  
  while(Serial.available()<=0);
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(unit, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void readAdc(int ind, gpio_num_t pin)
{
  AN_Pot_Raw[ind] = readADC_Cal(analogRead(pin));
  return;
}

void loop() {
  for (size_t i = 0; i < 4; i++)
  {
    switch (i)
    {
    case 0:
    readAdc(0, AN_Pot1);
    AN_Pot_Fil[0] = lp1.filt(AN_Pot_Raw[0]);
    break;
    case 1:
    readAdc(1, AN_Pot2);
    AN_Pot_Fil[1] = lp2.filt(AN_Pot_Raw[1]);
    break;
    case 2:
    readAdc(2, AN_Pot3);
    AN_Pot_Fil[2] = lp3.filt(AN_Pot_Raw[2]);
    break;
    case 3:
    readAdc(3, AN_Pot4);
    AN_Pot_Fil[3] = lp4.filt(AN_Pot_Raw[3]);
    break;
    }
  Serial.print(AN_Pot_Fil[i]); // Print Filtered Output
  Serial.print(' '); // Print Filtered Output
  }
  Serial.println();
  delay(5);
}