#include "audio_component.h"

class AudioRMS : public AudioComponent {
public:
    AudioRMS() : rmsLeft(0.0f), rmsRight(0.0f) {}
    virtual void process(AudioBuffer* block) override;

    float getRMSLeft() const { return rmsLeft; }
    float getRMSRight() const { return rmsRight; }  

private:
    float rmsLeft;  
    float rmsRight;
};

