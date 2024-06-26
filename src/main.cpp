#include <Arduino.h>

#define CLOCKRATE 80000000 /* Hz */
#define TIMERDIVIDER 4

#define OVER_SAMPLE_RATIO (16)
#define CYCLES (20)
#define NSAMPLES (OVER_SAMPLE_RATIO * CYCLES) // 321

#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS)

#define VOLTAGE_ADC_PIN (34)
#define CURRENT_ADC_PIN (35)

volatile int sampleCount = NSAMPLES;
volatile int voltageSamples[NSAMPLES];
volatile int currentSamples[NSAMPLES];

hw_timer_t *My_timer = NULL;

struct ElectricalMeasurements {
  double vrms;
  double irms;
  double realPower;
  double apparentPower;
  double powerFactor;
};

struct ElectricalMeasurements measurements;

void setupMeasurement();
struct ElectricalMeasurements makeMeasurement();

double amountMeasurements;
double sumVrms;
double sumIrms;
double sumRealPower;
double sumApparentPower;
double sumPowerFactor;

int countRmsMeasurements = 0;
double sumVoltageToSend = 0.0;
double sumCurrentToSend = 0.0;
double sumRealPowerToSend = 0.0;

void setup() {
  Serial.begin(115200);
  setupMeasurement();
}

void loop() {
  measurements = makeMeasurement();
  if (countRmsMeasurements <= 100) {
    sumVoltageToSend += measurements.vrms;
    sumCurrentToSend += measurements.irms;
    sumRealPowerToSend += measurements.realPower;

    countRmsMeasurements++;
    
    return;
  }
  Serial.print("Vrms: ");
  Serial.print(sumVoltageToSend / 100.0, 3);
  Serial.print(" (V); Irms: ");
  Serial.print(sumCurrentToSend / 100.0, 3);
  Serial.print(" (I); Prms: ");
  Serial.println(sumRealPowerToSend / 100.0, 3);

  sumVoltageToSend = 0;
  sumCurrentToSend = 0;
  sumRealPowerToSend = 0;

  countRmsMeasurements = 0;
}

void IRAM_ATTR onTimer() {
  if ((sampleCount >= 0) && (sampleCount < (NSAMPLES))) {
    voltageSamples[sampleCount] = analogRead(VOLTAGE_ADC_PIN);
    currentSamples[sampleCount] = analogRead(CURRENT_ADC_PIN);

    sampleCount++;
  }
}

void setupMeasurement() {
  My_timer = timerBegin(
    1,
    TIMERDIVIDER,
    true
  );

  timerAttachInterrupt(My_timer, &onTimer, true);

  float measureRatePerInterval = 1.0 / ( 60.0 * OVER_SAMPLE_RATIO);

  int amountTimeBetweenInterruption = (int)( measureRatePerInterval * CLOCKRATE / TIMERDIVIDER + 0.5);

  timerAlarmWrite(My_timer, amountTimeBetweenInterruption, true);

  timerAlarmEnable(My_timer);
}

void readAnalogSamples() {
  int waitDelay = 17 * CYCLES;
  sampleCount = 0;

  delay(waitDelay); // 340 ms
  if (sampleCount != NSAMPLES) {
    Serial.print("ADC processing is not working.");
  }

  timerWrite(My_timer, 0);
}

struct ElectricalMeasurements measureRms(int* voltageSamples, int* currentSamples, int nsamples) {
  float numberOfTurnsCT = 2.0;
  struct ElectricalMeasurements eletricMeasurements;
  // int32_t sumVoltageSamples = 0;
  // int32_t sumCurrentSamples = 0;

  // for (int i = 0; i < nsamples; i++) {
  //   Serial.print(voltageSamples[i]);
  //   Serial.print(" ");
  //   Serial.println(currentSamples[i]);
  //   sumVoltageSamples += voltageSamples[i];
  //   sumCurrentSamples += currentSamples[i];
  // }
  int offsetVoltage = 1851;
  int offsetCurrent = 1848;
  // int voltageMean = (int)(sumVoltageSamples / (int32_t)(nsamples));
  // int currentMean = (int)(sumCurrentSamples / (int32_t)(nsamples));

  float sumVoltage = 0;
  float sumCurrent = 0;
  float sumInstantaneousPower = 0;
  for (int i = 0; i < nsamples; i++) {
    int y_voltageNoOffset = voltageSamples[i] - offsetVoltage;
    int y_currentNoOffset = currentSamples[i] - offsetCurrent;

    float y_voltage = ((float)y_voltageNoOffset) * 3.3 / 4096.0;
    float y_current = ((float)y_currentNoOffset) * 3.3 / 4096.0;
    //y_current = y_current / numberOfTurnsCT;

    float y_instantaneousPower = y_voltage * y_current;
  
    sumVoltage += y_voltage * y_voltage;
    sumCurrent += y_current * y_current;
    sumInstantaneousPower += y_instantaneousPower;
  }

  float ym_voltage = sumVoltage / (float) nsamples;
  float ym_current = sumCurrent / (float) nsamples;
  float ym_realPower = sumInstantaneousPower / (float) nsamples;

  float vrmsSquared = sqrt(ym_voltage);
  float irmsSquared = sqrt(ym_current);
  float Vrms = vrmsSquared > 0.005 ? vrmsSquared : 0;
  float Irms = irmsSquared > 0.013 ? irmsSquared : 0;
  float realPower = ym_realPower;

  float vrmsCalibrated = Vrms > 0.0 ? (280.0 * Vrms) + 1.14 : 0.0;
  float irmsCalibrated = Irms > 0.0 ? (30.6 * Irms) + 0.00363 : 0.0;
  float realPowerCalibrated = (Vrms > 0.0 && Irms > 0.0) ? (8677.0 * realPower) + 2.98 : 0.0;

  eletricMeasurements.vrms = vrmsCalibrated;
  eletricMeasurements.irms = irmsCalibrated;
  eletricMeasurements.realPower = realPowerCalibrated;

  return eletricMeasurements;
}

struct ElectricalMeasurements makeMeasurement() {
  struct ElectricalMeasurements eletricMeasurements;

  readAnalogSamples();
  if (sampleCount == NSAMPLES) {
    eletricMeasurements = measureRms((int*) voltageSamples, (int*) currentSamples, NSAMPLES);
  }

  return eletricMeasurements;
}