 #ifndef filter_biquad_f_h_
 #define filter_biquad_f_h_
 
 #include <Arduino.h>     // github.com/PaulStoffregen/cores/blob/master/teensy4/Arduino.h
 #include <AudioStream.h> // github.com/PaulStoffregen/cores/blob/master/teensy4/AudioStream.h

 #define MAX_BIQUAD_STAGES 4
 #define STAGE_COEFFICIENTS 5

 typedef float sample_t;

 // Flat filter with unity gain. 
 static const sample_t identityCoefficients[STAGE_COEFFICIENTS] = {1.0, 0.0, 0.0, 0.0, 0.0};
 
 class AudioFilterBiquadFloat : public AudioStream
 {
 public:
     AudioFilterBiquadFloat(void) : AudioStream(1, inputQueueArray) {
         // by default, the filter will not pass anything
         for (int i=0; i<MAX_BIQUAD_STAGES * STAGE_COEFFICIENTS; i++) coeff[i] = 0;
         for (int i=0; i<MAX_BIQUAD_STAGES * 2; i++) state[i] = 0;
     }
     virtual void update(void);
 
     // Set the biquad coefficients directly
     void setCoefficients(uint32_t stage, const sample_t *coefficients) {        
         __disable_irq()
         for(int i = 0; i < STAGE_COEFFICIENTS; i++) {
             coeff[stage * STAGE_COEFFICIENTS + i] = coefficients[i];
         }
         if(stage + 1 > num_stages) {
             num_stages = stage + 1;
         }
         __enable_irq()
         // Serial.printf("num_stages = %d\n", num_stages);
     }

     void setSosCoefficients(uint32_t stages, const sample_t *sos) {
         __disable_irq()
         for(uint32_t i = 0; i < stages * STAGE_COEFFICIENTS; i++) {
            coeff[i] = sos[i]; 
         }
         num_stages = stages;
         __enable_irq()
     }

     const float* getCoefficients(uint32_t stage) {
        // Return identity filter if stage isn't active
        return stage < num_stages ? &coeff[stage * STAGE_COEFFICIENTS] : identityCoefficients;
     }
 
     // Compute common filter functions
     // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
     void setLowpass(uint32_t stage, sample_t frequency, sample_t q = 0.7071f) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         sample_t alpha = sinW0 / ((sample_t)q * 2.0);
         sample_t cosW0 = cos(w0);
         sample_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = ((1.0 - cosW0) / 2.0) * scale;
         /* b1 */ coef[1] = (1.0 - cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setHighpass(uint32_t stage, sample_t frequency, sample_t q = 0.7071) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         sample_t alpha = sinW0 / ((sample_t)q * 2.0);
         sample_t cosW0 = cos(w0);
         sample_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = ((1.0 + cosW0) / 2.0) * scale;
         /* b1 */ coef[1] = -(1.0 + cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a0 */ coef[3] = 1.0;
         /* a1 */ coef[4] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[5] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setBandpass(uint32_t stage, sample_t frequency, sample_t q = 1.0) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         sample_t alpha = sinW0 / ((sample_t)q * 2.0);
         sample_t cosW0 = cos(w0);
         sample_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = alpha * scale;
         /* b1 */ coef[1] = 0;
         /* b2 */ coef[2] = (-alpha) * scale;
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setNotch(uint32_t stage, sample_t frequency, sample_t q = 1.0) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         sample_t alpha = sinW0 / ((sample_t)q * 2.0);
         sample_t cosW0 = cos(w0);
         sample_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = scale;
         /* b1 */ coef[1] = (-2.0 * cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setLowShelf(uint32_t stage, sample_t frequency, sample_t gain, sample_t slope = 1.0f) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t a = pow(10.0, gain/40.0f);
         Serial.printf("LowShelf Gain a = %f\n", a);
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         //sample_t alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
         sample_t cosW0 = cos(w0);
         //generate three helper-values (intermediate results):
         sample_t ss = (a*a+1.0)*(1.0/(sample_t)slope-1.0)+2.0*a;
         if(ss < 0.0) {
            // Avoid taking the square root of a negative number
            return; // invalid parameter combination
         }
         sample_t sinsq = sinW0 * sqrt( ss );
         sample_t aMinus = (a-1.0)*cosW0;
         sample_t aPlus = (a+1.0)*cosW0;
         sample_t scale = 1.0 / ( (a+1.0) + aMinus + sinsq);
         #ifdef VERBOSE
         // Print all pre-calculated values for debugging
         Serial.printf("LowShelf Filter Stage %d:\n", stage);
            Serial.printf("  a = %f\n", a);
            Serial.printf("  w0 = %f\n", w0);
            Serial.printf("  sinW0 = %f\n", sinW0);
            Serial.printf("  cosW0 = %f\n", cosW0);
            Serial.printf("  sinsq = %f\n", sinsq);
            Serial.printf("  aMinus = %f\n", aMinus);
            Serial.printf("  aPlus = %f\n", aPlus);
            Serial.printf("  scale = %f\n", scale);
        #endif

         /* b0 */ coef[0] =		a *	( (a+1.0) - aMinus + sinsq	) * scale;
         /* b1 */ coef[1] =  2.0*a * ( (a-1.0) - aPlus  			) * scale;
         /* b2 */ coef[2] =		a * ( (a+1.0) - aMinus - sinsq 	) * scale;
         /* a1 */ coef[3] = -2.0*	( (a-1.0) + aPlus			) * scale;
         /* a2 */ coef[4] =  		( (a+1.0) + aMinus - sinsq	) * scale;
         #ifdef VERBOSE
         Serial.printf("LowShelf Coefficients for stage %d:\n", stage);
         for(int i=0; i<STAGE_COEFFICIENTS; i++) {
            Serial.printf("  coef[%d] = %f\n", i, coef[i]);
         }  
         #endif
         Serial.println();
         setCoefficients(stage, coef);
     }
     void setHighShelf(uint32_t stage, sample_t frequency, sample_t gain, sample_t slope = 1.0f) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t a = pow(10.0, gain/40.0f);
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         //sample_t alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
         sample_t cosW0 = cos(w0);
         //generate three helper-values (intermediate results):
         sample_t ss = (a*a+1.0)*(1.0/(sample_t)slope-1.0)+2.0*a;
         if(ss < 0.0) {
            // Avoid taking the square root of a negative number
            return; 
         }
         sample_t sinsq = sinW0 * sqrt(ss);
         sample_t aMinus = (a-1.0)*cosW0;
         sample_t aPlus = (a+1.0)*cosW0;
         sample_t scale = 1.0 / ( (a+1.0) - aMinus + sinsq);
         /* b0 */ coef[0] =		a *	( (a+1.0) + aMinus + sinsq	) * scale;
         /* b1 */ coef[1] = -2.0*a * ( (a-1.0) + aPlus  			) * scale;
         /* b2 */ coef[2] =		a * ( (a+1.0) + aMinus - sinsq 	) * scale;
         /* a1 */ coef[3] =  2.0*	( (a-1.0) - aPlus			) * scale;
         /* a2 */ coef[4] =  		( (a+1.0) - aMinus - sinsq	) * scale;
         setCoefficients(stage, coef);
     }

     void setPeakingEQ(uint32_t stage, sample_t frequency, sample_t q, sample_t gain) {
         sample_t coef[STAGE_COEFFICIENTS];
         sample_t a = pow(10.0, gain/40.0f);
         sample_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         sample_t sinW0 = sin(w0);
         sample_t alpha = sinW0 / (q * 2.0f);
         sample_t cosW0 = cos(w0);
         sample_t scale = 1.0 / (1.0 + alpha / a);
         /* b0 */ coef[0] = (1.0 + alpha * a) * scale;
         /* b1 */ coef[1] = (-2.0 * cosW0) * scale; 
         /* b2 */ coef[2] = (1.0 - alpha * a) * scale;
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha / a) * scale; 
         //for(int i=0; i<STAGE_COEFFICIENTS; i++) {
            //Serial.printf("coef[%d] = %f\n", i, coef[i]);
         //}
         setCoefficients(stage, coef);
     }

     void bypass(uint32_t stage) {
        setCoefficients(stage, identityCoefficients);
    }

 //private:
     sample_t coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
     sample_t state[2 * MAX_BIQUAD_STAGES];
     audio_block_t *inputQueueArray[1];
     uint32_t num_stages = 0; // number of stages in use
 };
 
 #endif
 