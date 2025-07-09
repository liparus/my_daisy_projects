//
//
// HUOM: Koodi ei käänny, mutta antaa viitteitä
//       efektiketjun toteutukseen

/*
#include "daisy_pod.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyPod pod;
DelayLine<float, 48000> delay;  // Initialize a delay line
Bitcrush crush;  // Initialize a bitcrusher
bool isBitcrusherSelected = false;
bool isEffectEnabled = true;  // Initialize with the effect enabled

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        float dry = in[0][i];  // Mono input
        float bitcrushed = 0.0f;
        float delayed = 0.0f;

        // Check if the effect is enabled
        if (isEffectEnabled)
        {
            // Apply the bitcrusher effect first if selected
            if (isBitcrusherSelected)
            {
                float tone = pod.knob1.Process();
                float crushAmount = pod.knob2.Process();
                crush.Init(0.5f, tone, crushAmount);
                bitcrushed = crush.Process(dry);
            }
            else
            {
                bitcrushed = dry;
            }

            // Apply the delay effect
            float speed = pod.knob1.Process();
            float time = pod.knob2.Process();
            delay.SetSpeed(speed);
            delay.SetDelay(time);
            delayed = delay.Read(bitcrushed);
        }
        else
        {
            // Bypass the effect, send the dry input to the output
            delayed = dry;
        }

        // Output the result
        out[0][i] = delayed;
    }
}

int main()
{
    pod.Init();
    float sample_rate = pod.AudioSampleRate();
    delay.Init(0.5f * sample_rate);
    crush.Init(0.5f);

    while (1)
    {
        pod.UpdateAnalogControls();
        
        // Toggle between bitcrusher and delay effects
        if (pod.switches[SW_1].RisingEdge())
        {
            isBitcrusherSelected = !isBitcrusherSelected;
        }

        // Toggle effect on/off with a push button
        if (pod.switches[SW_2].RisingEdge())
        {
            isEffectEnabled = !isEffectEnabled;
        }

        pod.ProcessAnalogControls();
        pod.ProcessDigitalControls();
    }
}
*/


#include "daisysp.h"
#include "daisy_pod.h"

// Set max delay time to 0.75 of samplerate.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)
#define REV 0
#define DEL 1
#define CRU 2

using namespace daisysp;
using namespace daisy;

static DaisyPod pod;

static ReverbSc rev;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dell;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delr;
static Tone tone;
static Parameter deltime, cutoffParam, crushrate;
int mode = REV;

float currentDelay, feedback, delayTarget, cutoff;

int crushmod, crushcount;
float crushsl, crushsr, drywet;

// New global variable for effect bypass
bool isEffectBypassed = false;

// Helper functions
void Controls();

void GetReverbSample(float &outl, float &outr, float inl, float inr);

void GetDelaySample(float &outl, float &outr, float inl, float inr);

void GetCrushSample(float &outl, float &outr, float inl, float inr);

void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t size)
{
    float outl, outr, inl, inr;

    Controls();

    // Temp variables to hold intermediate processing results
    float crushOutL, crushOutR, delayOutL, delayOutR;

    // Audio processing loop
    for (size_t i = 0; i < size; i += 2)
    {
        inl = in[i];
        inr = in[i + 1];

        // Check if the effect is bypassed
        if (isEffectBypassed)
        {
            // Pass the input directly to the output without processing
            outl = inl;
            outr = inr;
        }
        else
        {
            // Process bitcrusher
            GetCrushSample(crushOutL, crushOutR, inl, inr);

            // Process delay with bitcrushed input
            GetDelaySample(delayOutL, delayOutR, crushOutL, crushOutR);

            // Process reverb with delayed input
            GetReverbSample(outl, outr, delayOutL, delayOutR);
        }

        // Output the final processed audio
        out[i] = outl;
        out[i + 1] = outr;
    }
}

int main(void)
{
    // initialize pod hardware and oscillator daisysp module
    float sample_rate;

    // Inits and sample rate
    pod.Init();
    pod.SetAudioBlockSize(4);
    sample_rate = pod.AudioSampleRate();
    rev.Init(sample_rate);
    dell.Init();
    delr.Init();
    tone.Init(sample_rate);

    // set parameters
    deltime.Init(pod.knob1, sample_rate * .05, MAX_DELAY, deltime.LOGARITHMIC);
    cutoffParam.Init(pod.knob1, 500, 20000, cutoffParam.LOGARITHMIC);
    crushrate.Init(pod.knob2, 1, 50, crushrate.LOGARITHMIC);

    // reverb parameters
    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    // delay parameters
    currentDelay = delayTarget = sample_rate * 0.75f;
    dell.SetDelay(currentDelay);
    delr.SetDelay(currentDelay);

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while (1) {}
}

void UpdateKnobs(float &k1, float &k2)
{
    k1 = pod.knob1.Process();
    k2 = pod.knob2.Process();

    switch (mode)
    {
    case REV:
        drywet = k1;
        rev.SetFeedback(k2);
        break;
    case DEL:
        delayTarget = deltime.Process();
        feedback = k2;
        break;
    case CRU:
        cutoff = cutoffParam.Process();
        tone.SetFreq(cutoff);
        crushmod = (int)crushrate.Process();
    }
}

void UpdateEncoder()
{
    // Use the encoder to scroll through effects
    mode = mode + pod.encoder.Increment();
    mode = (mode % 3 + 3) % 3;

    // Check if the encoder is pressed
    if (pod.encoder.Pressed())
    {
        // Toggle effect bypass state
        isEffectBypassed = !isEffectBypassed;
    }
}

void UpdateLeds(float k1, float k2)
{
    pod.led1.Set(
        k1 * (mode == 2), k1 * (mode == 1), k1 * (mode == 0 || mode == 2));
    pod.led2.Set(
        k2 * (mode == 2), k2 * (mode == 1), k2 * (mode == 0 || mode == 2));

    pod.UpdateLeds();
}

void Controls()
{
    float k1, k2;
    delayTarget = feedback = drywet = 0;

    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    UpdateKnobs(k1, k2);

    UpdateEncoder();

    UpdateLeds(k1, k2);
}

void GetReverbSample(float &outl, float &outr, float inl, float inr)
{
    rev.Process(inl, inr, &outl, &outr);
    outl = drywet * outl + (1 - drywet) * inl;
    outr = drywet * outr + (1 - drywet) * inr;
}

void GetDelaySample(float &outl, float &outr, float inl, float inr)
{
    fonepole(currentDelay, delayTarget, .00007f);
    delr.SetDelay(currentDelay);
    dell.SetDelay(currentDelay);
    outl = dell.Read();
    outr = delr.Read();

    dell.Write((feedback * outl) + inl);
    outl = (feedback * outl) + ((1.0f - feedback) * inl);

    delr.Write((feedback * outr) + inr);
    outr = (feedback * outr) + ((1.0f - feedback) * inr);
}

void GetCrushSample(float &outl, float &outr, float inl, float inr)
{
    crushcount++;
    crushcount %= crushmod;
    if (crushcount == 0)
    {
        crushsr = inr;
        crushsl = inl;
    }
    outl = tone.Process(crushsl);
    outr = tone.Process(crushsr);
}
