#include "audio_component.h"

class AudioRMS : public AudioComponent {
public:
    AudioRMS() : rmsLeft(0.0f), rmsRight(0.0f) {}
    virtual void process(AudioBuffer* block) override;

    float getRMSLeft() const { return rmsLeft; }
    float getRMSRight() const { return rmsRight; }  
    float getRMSCombined() const { return (rmsLeft + rmsRight) / 2.0f; }

private:
    float rmsLeft;  
    float rmsRight;
};

