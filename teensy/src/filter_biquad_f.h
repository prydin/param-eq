

 #ifndef filter_biquad_f_h_
 #define filter_biquad_f_h_
 
 #include <Arduino.h>     // github.com/PaulStoffregen/cores/blob/master/teensy4/Arduino.h
 #include <AudioStream.h> // github.com/PaulStoffregen/cores/blob/master/teensy4/AudioStream.h

 #define MAX_BIQUAD_STAGES 4
 #define STAGE_COEFFICIENTS 5

 typedef float biquad_num_t;
 
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
     void setCoefficients(uint32_t stage, const biquad_num_t *coefficients) {        
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

     void setSosCoefficients(uint32_t stages, const biquad_num_t *sos) {
         __disable_irq()
         for(uint32_t i = 0; i < stages * STAGE_COEFFICIENTS; i++) {
            coeff[i] = sos[i]; 
         }
         num_stages = stages;
         __enable_irq()
     }

     const float* getCoefficients(uint32_t stage) {
        return &coeff[stage * STAGE_COEFFICIENTS];
     }
 
     // Compute common filter functions
     // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
     void setLowpass(uint32_t stage, biquad_num_t frequency, biquad_num_t q = 0.7071f) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         biquad_num_t alpha = sinW0 / ((biquad_num_t)q * 2.0);
         biquad_num_t cosW0 = cos(w0);
         biquad_num_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = ((1.0 - cosW0) / 2.0) * scale;
         /* b1 */ coef[1] = (1.0 - cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setHighpass(uint32_t stage, biquad_num_t frequency, biquad_num_t q = 0.7071) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         biquad_num_t alpha = sinW0 / ((biquad_num_t)q * 2.0);
         biquad_num_t cosW0 = cos(w0);
         biquad_num_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = ((1.0 + cosW0) / 2.0) * scale;
         /* b1 */ coef[1] = -(1.0 + cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a0 */ coef[3] = 1.0;
         /* a1 */ coef[4] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[5] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setBandpass(uint32_t stage, biquad_num_t frequency, biquad_num_t q = 1.0) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         biquad_num_t alpha = sinW0 / ((biquad_num_t)q * 2.0);
         biquad_num_t cosW0 = cos(w0);
         biquad_num_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = alpha * scale;
         /* b1 */ coef[1] = 0;
         /* b2 */ coef[2] = (-alpha) * scale;
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setNotch(uint32_t stage, biquad_num_t frequency, biquad_num_t q = 1.0) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         biquad_num_t alpha = sinW0 / ((biquad_num_t)q * 2.0);
         biquad_num_t cosW0 = cos(w0);
         biquad_num_t scale = 1.0 / (1.0 + alpha);
         /* b0 */ coef[0] = scale;
         /* b1 */ coef[1] = (-2.0 * cosW0) * scale;
         /* b2 */ coef[2] = coef[0];
         /* a1 */ coef[3] = (-2.0 * cosW0) * scale;
         /* a2 */ coef[4] = (1.0 - alpha) * scale;
         setCoefficients(stage, coef);
     }
     void setLowShelf(uint32_t stage, biquad_num_t frequency, biquad_num_t gain, biquad_num_t slope = 1.0f) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t a = pow(10.0, gain/40.0f);
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         //biquad_num_t alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
         biquad_num_t cosW0 = cos(w0);
         //generate three helper-values (intermediate results):
         biquad_num_t sinsq = sinW0 * sqrt( (pow(a,2.0)+1.0)*(1.0/(biquad_num_t)slope-1.0)+2.0*a );
         biquad_num_t aMinus = (a-1.0)*cosW0;
         biquad_num_t aPlus = (a+1.0)*cosW0;
         biquad_num_t scale = 1.0 / ( (a+1.0) + aMinus + sinsq);
         /* b0 */ coef[0] =		a *	( (a+1.0) - aMinus + sinsq	) * scale;
         /* b1 */ coef[1] =  2.0*a * ( (a-1.0) - aPlus  			) * scale;
         /* b2 */ coef[2] =		a * ( (a+1.0) - aMinus - sinsq 	) * scale;
         /* a1 */ coef[3] = -2.0*	( (a-1.0) + aPlus			) * scale;
         /* a2 */ coef[4] =  		( (a+1.0) + aMinus - sinsq	) * scale;
         setCoefficients(stage, coef);
     }
     void setHighShelf(uint32_t stage, biquad_num_t frequency, biquad_num_t gain, biquad_num_t slope = 1.0f) {
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t a = pow(10.0, gain/40.0f);
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         //biquad_num_t alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
         biquad_num_t cosW0 = cos(w0);
         //generate three helper-values (intermediate results):
         biquad_num_t sinsq = sinW0 * sqrt( (pow(a,2.0)+1.0)*(1.0/(biquad_num_t)slope-1.0)+2.0*a );
         biquad_num_t aMinus = (a-1.0)*cosW0;
         biquad_num_t aPlus = (a+1.0)*cosW0;
         biquad_num_t scale = 1.0 / ( (a+1.0) - aMinus + sinsq);
         /* b0 */ coef[0] =		a *	( (a+1.0) + aMinus + sinsq	) * scale;
         /* b1 */ coef[1] = -2.0*a * ( (a-1.0) + aPlus  			) * scale;
         /* b2 */ coef[2] =		a * ( (a+1.0) + aMinus - sinsq 	) * scale;
         /* a1 */ coef[3] =  2.0*	( (a-1.0) - aPlus			) * scale;
         /* a2 */ coef[4] =  		( (a+1.0) - aMinus - sinsq	) * scale;
         setCoefficients(stage, coef);
     }

     void setPeakingEQ(uint32_t stage, biquad_num_t frequency, biquad_num_t q, biquad_num_t gain) {
        /*
  A=math.sqrt(10**(G/20))
    omega_c = 2*math.pi*fc/fs
    cos_omega_c = math.cos(omega_c)
    sin_omega_c = math.sin(omega_c)
    alpha = sin_omega_c /(2*Q)
    beta = math.sqrt(A)/Q

          A, omega_c, cos_omega_c, sin_omega_c, alpha, beta = precalc(fc, fs, Q, G)
    b0 = 1 + alpha*A
    b1 = -2*cos_omega_c
    b2 = 1 - alpha*A
    a0 = 1 + alpha/A
    a1 = -2*cos_omega_c
    a2 = 1 - alpha/A
    return [b0/a0, b1/a0, b2/a0], [1, a1/a0, a2/a0]

        
        */
       //Serial.printf("setPeakingEQ: stage=%d, freq=%f, q=%f, gain=%f\n", stage, frequency, q, gain);
         biquad_num_t coef[STAGE_COEFFICIENTS];
         biquad_num_t a = pow(10.0, gain/40.0f);
         biquad_num_t w0 = frequency * (2.0f * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT);
         biquad_num_t sinW0 = sin(w0);
         biquad_num_t alpha = sinW0 / (q * 2.0f);
         biquad_num_t cosW0 = cos(w0);
         biquad_num_t scale = 1.0 / (1.0 + alpha / a);
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

 //private:
     biquad_num_t coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
     biquad_num_t state[2 * MAX_BIQUAD_STAGES];
     audio_block_t *inputQueueArray[1];
     uint32_t num_stages = 0; // number of stages in use
 };
 
 #endif
 