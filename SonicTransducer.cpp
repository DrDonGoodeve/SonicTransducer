/******************************************************************************
 * SonicTransducer.cpp
 * 
 * Main program file (and support routines) for RP PICO Sonic Transducer
 * controller board. This support 10x output PWM plus heartbeat PWM and
 * two momentary switches to key between multiple patterns. Power on is
 * into 'Sonic Transducer (pin everyone to the floor)' mode.
 * Because Riff Raff needs some fucking cool tools...
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "pico/stdlib.h"
#include <stdio.h>
#include <list>
#include <math.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

// Defines
//-----------------------------------------------------------------------------
#define kPicoClockHz        (125.0e6f)
#define kClockDivider       (4.0f)
#define kPWMClockHz         (kPicoClockHz / kClockDivider)
#define kPWMClkSec          (1.0f / kPWMClock)
#define kPWMCountMax        (0x10000)
#define kPWMFullFreq        (kPWMClockHz / (float)kPWMCountMax)
#define kPWMWrapDuration    (1.0f / kPWMFullFreq)
#define kPWMWrapHz          (kPWMFullFreq)

/*#define kTop        (0)
#define kRow1c1     (1)
#define kRow1c2     (2)
#define kRow1c3     (3)
#define kRow2c1     (4)
#define kRow2c2     (6)
#define kRow2c3     (5)
#define kRow3c1     (7)
#define kRow3c2     (8)
#define kRow3c3     (9)
*/
#define kTop        (9)
#define kRow1c1     (5)
#define kRow1c2     (7)
#define kRow1c3     (8)
#define kRow2c1     (3)
#define kRow2c2     (4)
#define kRow2c3     (6)
#define kRow3c1     (0)
#define kRow3c2     (1)
#define kRow3c3     (2)
#define kAliveLED   (PICO_DEFAULT_LED_PIN)  // Note same as kTop

#define kBlueButton    (16)
#define kGreenButton   (17)
#define kModeSwitch    (18)
#define kFobSwitch     (19)

#define kDoublePressUsec   (300000)
#define kHoldTime          (700000)

class _PWMManager {
    public:
        virtual void ISRUpdate(void) = 0;
};
static _PWMManager *spPWMManager = nullptr;

static
void _stepPWM(void) {
    if (spPWMManager != nullptr) {
        spPWMManager->ISRUpdate();
    }
}

class PWMManager : public _PWMManager {
    public:
        class PWMStep {
            public:
                float mfDurationSec;
                float mfTargetLevel;
        };

        class PWMProgram {
            public:
                const PWMStep *mpSequence;
                uint muSteps;
                PWMProgram(const PWMStep *pSequence, uint uSteps) :
                    mpSequence(pSequence), muSteps(uSteps) {
                }
        };

    private:
        class PWM {
            private:
                uint muGPIO;
                std::list<PWMStep> mlNext;
                bool mbRepeat;
                float mfCurrentLevel;
                float mfTargetLevel;
                uint muStepsRemaining;

            public:
                PWM(uint uGPIO) :
                    muGPIO(uGPIO),
                    mfCurrentLevel(0.0f), mfTargetLevel(0.0f), muStepsRemaining(0.0f) {
                    gpio_set_function(muGPIO, GPIO_FUNC_PWM);
                    pwm_set_gpio_level(muGPIO, 0x0);    // Off
                    // Get some sensible defaults for the slice configuration. By default, the
                    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
                    pwm_config cPWMConfig(pwm_get_default_config());
                    // Set divider, reduces counter clock to sysclock/this value
                    pwm_config_set_clkdiv(&cPWMConfig, (float)kClockDivider);
                    pwm_init(pwm_gpio_to_slice_num(muGPIO), &cPWMConfig, true);
                }

                ~PWM() {
                }

                uint getGPIO(void) const {
                    return muGPIO;
                }

                void setProgram(const PWMStep *pSteps, uint uSteps, int iOffset, bool bRepeat) {
                    std::list<PWMStep> lProgram;
                    int iCorrectedOffset(iOffset%uSteps);
                    //printf("iOffset=%d, iCorrectedOffset=%d, uSteps=%d\r\n", iOffset, iCorrectedOffset, uSteps);
                    for(uint i=0; i<uSteps; i++) {
                        lProgram.push_back(pSteps[(i+iCorrectedOffset)%uSteps]);
                    }
                    
                    uint32_t cStatus(save_and_disable_interrupts());
                    mlNext = lProgram;
                    mbRepeat = bRepeat;
                    muStepsRemaining = 0;
                    restore_interrupts(cStatus);
                }

                void update(void) {
                    if (0 == muStepsRemaining) {
                        if (false == mlNext.empty()) {
                            // Grab next step
                            PWMStep cStep(mlNext.front());
                            mlNext.pop_front();
                            if (true == mbRepeat) {
                                mlNext.push_back(cStep);
                            }

                            // Interpret it
                            muStepsRemaining = (uint)roundf(kPWMWrapHz * cStep.mfDurationSec);
                            mfTargetLevel = cStep.mfTargetLevel;
                            //printf("Target level %.2f in steps %d (kWrapsPerSec=%f)\r\n", mfTargetLevel, muStepsRemaining, kPWMWrapHz);
                        }
                    }
                    if (muStepsRemaining > 0) {
                        float fDiff(mfTargetLevel - mfCurrentLevel);
                        float fDelta(fDiff / (float)muStepsRemaining);
                        mfCurrentLevel += fDelta;
                        muStepsRemaining--;

                        uint uRangedLevel(mfCurrentLevel * 255.0f);
                        uRangedLevel = (uRangedLevel > 255)?255:uRangedLevel;
                        pwm_set_gpio_level(muGPIO, uRangedLevel*uRangedLevel);
                    }
                }
        };
        
        uint muPWMIRQSlice;
        std::list<PWM> mlPWMs;

    public:
        PWMManager(const uint *pGPIOSet, uint uGPIOs) {
            if (0 == uGPIOs) {
                return;
            }
            for(uint i=0; i<uGPIOs; i++) {
                mlPWMs.push_back(PWM(pGPIOSet[i]));

            }

            uint uBasePWM(pGPIOSet[0]);
            // Figure out which slice we just connected to the uBasePWM. We use this block
            // as the interrupt source (on wrap). Note there is just one PWM IRQ.
            muPWMIRQSlice = pwm_gpio_to_slice_num(uBasePWM);

            // Mask our slice's IRQ output into the PWM block's single interrupt line,
            // register the interrupt handler and enable it.
            pwm_clear_irq(muPWMIRQSlice);
            pwm_set_irq_enabled(muPWMIRQSlice, true);
            irq_set_exclusive_handler(PWM_IRQ_WRAP, _stepPWM);
            irq_set_enabled(PWM_IRQ_WRAP, true);
        }

        ~PWMManager() {
            pwm_set_irq_enabled(muPWMIRQSlice, false);
        }

        void setProgram(uint uGPIO, const PWMStep *pSteps, uint uSteps, int iOffset=0, bool bRepeat=true) {
            for(std::list<PWM>::iterator cPWM = mlPWMs.begin(); cPWM != mlPWMs.end(); ++cPWM) {
                if ((*cPWM).getGPIO() == uGPIO) {
                    (*cPWM).setProgram(pSteps, uSteps, iOffset, bRepeat);
                    break;
                }
            }
        }

        void setProgram(uint uGPIO, const PWMProgram &cProgram, int iOffset=0, bool bRepeat=true) {
            setProgram(uGPIO, cProgram.mpSequence, cProgram.muSteps, iOffset, bRepeat);
        }

        void ISRUpdate(void) {
            pwm_clear_irq(muPWMIRQSlice);
            for(std::list<PWM>::iterator cPWM = mlPWMs.begin(); cPWM != mlPWMs.end(); ++cPWM) {
                (*cPWM).update();
            }
        }
};

#define _arraysize(a)  (sizeof(a)/sizeof(a[0]))



// LED output programs
static const PWMManager::PWMStep pOffSequence[] = {{0.01f, 0.0f}};
static const PWMManager::PWMProgram cOffProgram(pOffSequence, _arraysize(pOffSequence));

static const PWMManager::PWMStep pLightsTestSequence[] = {
    {0.3f, 0.3f},
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f},
    {0.3f, 0.0f}, 
    {0.3f, 0.0f}, 
    {0.3f, 0.0f},
};
static const PWMManager::PWMProgram cLightsTestProgram(pLightsTestSequence, _arraysize(pLightsTestSequence));

static const PWMManager::PWMStep pDimPulseSequence[] = {
    {0.4f, 0.1f},
    {0.4f, 0.0f}, 
    {0.4f, 0.0f}, 
    {0.4f, 0.0f}, 
    {0.4f, 0.0f},
    {0.4f, 0.0f}, 
    {0.4f, 0.0f}, 
    {0.4f, 0.0f},
};
static const PWMManager::PWMProgram cDimPulseProgram(pDimPulseSequence, _arraysize(pDimPulseSequence));

static const PWMManager::PWMStep pReactorSequence1[] = {
    {0.2f, 0.0f}, 
    {0.2f, 0.0f}, 
    {0.2f, 0.0f},
    {0.2f, 0.0f}, 
    {0.2f, 0.0f}, 
    {0.2f, 0.0f},
    {0.2f, 0.2f}, 
    {0.2f, 0.5f}, 
    {0.2f, 0.8f}, 
    {0.2f, 0.5f}, 
    {0.2f, 0.2f}, 
};
static const PWMManager::PWMProgram cReactorProgram1(pReactorSequence1, _arraysize(pReactorSequence1));

static const PWMManager::PWMStep pReactorSequence2[] = {
    {0.1f, 0.0f}, 
    {0.1f, 0.0f}, 
    {0.1f, 0.0f},
    {0.1f, 0.0f}, 
    {0.1f, 0.0f}, 
    {0.1f, 0.0f},
    {0.1f, 0.2f}, 
    {0.1f, 0.5f}, 
    {0.1f, 1.0f}, 
    {0.1f, 0.5f}, 
    {0.1f, 0.2f}, 
};
static const PWMManager::PWMProgram cReactorProgram2(pReactorSequence2, _arraysize(pReactorSequence2));

static const PWMManager::PWMStep pReactorSequence3[] = {
    {0.05f, 0.0f}, 
    {0.05f, 0.0f}, 
    {0.05f, 0.0f},
    {0.05f, 0.0f}, 
    {0.05f, 0.0f}, 
    {0.05f, 0.0f},
    {0.05f, 0.2f}, 
    {0.05f, 0.5f}, 
    {0.05f, 1.0f}, 
    {0.05f, 0.5f}, 
    {0.05f, 0.2f}, 
};
static const PWMManager::PWMProgram cReactorProgram3(pReactorSequence3, _arraysize(pReactorSequence3));

static const PWMManager::PWMStep pIndicatorsFlash[] = {
    {0.05f, 1.0f},
    {0.10f, 1.0f}, 
    {0.20f, 0.0f}, 
    {0.02f, 0.0f},
    {0.05f, 1.0f}, 
    {0.10f, 1.0f}, 
    {0.20f, 0.0f}, 
    {0.50f, 0.0f},
};
static const PWMManager::PWMProgram cIndicatorsProgram(pIndicatorsFlash, _arraysize(pIndicatorsFlash));

static const PWMManager::PWMStep pSTSequence1[] = {
    {0.2f, 0.0f}, 
    {0.2f, 0.2f},
    {0.2f, 0.4f}, 
    {0.2f, 0.8f}, 
    {0.2f, 1.0f}, 
    {0.2f, 0.8f},
    {0.2f, 0.4f},
    {0.2f, 0.2f},
    {0.2f, 0.0f},
};
static const PWMManager::PWMProgram cSTProgram1(pSTSequence1, _arraysize(pSTSequence1));

static const PWMManager::PWMStep pSTSequence2[] = {
    {0.1f, 0.0f}, 
    {0.1f, 0.2f},
    {0.1f, 0.4f}, 
    {0.1f, 0.8f}, 
    {0.1f, 1.0f}, 
    {0.1f, 0.8f},
    {0.1f, 0.4f},
    {0.1f, 0.2f},
    {0.1f, 0.0f},
};
static const PWMManager::PWMProgram cSTProgram2(pSTSequence2, _arraysize(pSTSequence2));

static const PWMManager::PWMStep pSTSequence3[] = {
    {0.05f, 0.0f}, 
    {0.05f, 0.2f},
    {0.05f, 0.4f}, 
    {0.05f, 0.8f}, 
    {0.05f, 1.0f}, 
    {0.05f, 0.8f},
    {0.05f, 0.4f},
    {0.05f, 0.2f},
    {0.05f, 0.0f},
};
static const PWMManager::PWMProgram cSTProgram3(pSTSequence2, _arraysize(pSTSequence2));

static const PWMManager::PWMStep pTBSequence1[] = {
    {0.4f, 0.1f}, 
    {0.4f, 0.2f},
    {0.4f, 0.4f},
    {0.4f, 0.5f}, 
    {0.4f, 0.4f},
    {0.4f, 0.2f}, 
    {0.4f, 0.1f},
};
static const PWMManager::PWMProgram cTBProgram12(pTBSequence1, _arraysize(pTBSequence1));

static const PWMManager::PWMStep pTBSequence3[] = {
    {0.2f, 0.1f}, 
    {0.2f, 0.0f},
    {0.2f, 0.3f},
    {0.2f, 0.0f}, 
    {0.2f, 0.7f},
    {0.2f, 0.0f}, 
    {0.2f, 1.0f},
    {0.2f, 0.0f},
    {0.2f, 0.0f},
};
static const PWMManager::PWMProgram cTBProgram3(pTBSequence3, _arraysize(pTBSequence3));


static PWMManager *spFullPWMManager = nullptr;

// Light step sequences
//-----------------------------------------------------------------------------
typedef void (*LEDLoadFn)(void);

static void _allOff(void) {
    spFullPWMManager->setProgram(kTop, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow1c1, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow1c2, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow1c3, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow2c1, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow2c2, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow2c3, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow3c1, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow3c2, cOffProgram, 0);
    spFullPWMManager->setProgram(kRow3c3, cOffProgram, 0);
}

// Reactor steps
//-------------------------------------
static void _reactorStep0(void) {
    printf("reactor step 0\r\n");
    _allOff();
    spFullPWMManager->setProgram(kRow3c1, cDimPulseProgram, 0);
}
static void _reactorStep1(void) {
    printf("reactor step 1\r\n");
    spFullPWMManager->setProgram(kTop, cReactorProgram1, 3);
    spFullPWMManager->setProgram(kRow1c3, cReactorProgram1, 3);
    spFullPWMManager->setProgram(kRow2c3, cReactorProgram1, 3);
    spFullPWMManager->setProgram(kRow3c3, cReactorProgram1, 3);
    spFullPWMManager->setProgram(kRow1c2, cReactorProgram1, 2);
    spFullPWMManager->setProgram(kRow2c2, cReactorProgram1, 2);
    spFullPWMManager->setProgram(kRow3c2, cReactorProgram1, 2);
    spFullPWMManager->setProgram(kRow1c1, cReactorProgram1, 1);
    spFullPWMManager->setProgram(kRow3c1, cReactorProgram1, 1);
    spFullPWMManager->setProgram(kRow2c1, cReactorProgram1, 0);
}
static void _reactorStep2(void) {
    printf("reactor step 2\r\n");
    spFullPWMManager->setProgram(kTop, cReactorProgram2, 3);
    spFullPWMManager->setProgram(kRow1c3, cReactorProgram2, 3);
    spFullPWMManager->setProgram(kRow2c3, cReactorProgram2, 3);
    spFullPWMManager->setProgram(kRow3c3, cReactorProgram2, 3);
    spFullPWMManager->setProgram(kRow1c2, cReactorProgram2, 2);
    spFullPWMManager->setProgram(kRow2c2, cReactorProgram2, 2);
    spFullPWMManager->setProgram(kRow3c2, cReactorProgram2, 2);
    spFullPWMManager->setProgram(kRow1c1, cReactorProgram2, 1);
    spFullPWMManager->setProgram(kRow3c1, cReactorProgram2, 1);
    spFullPWMManager->setProgram(kRow2c1, cReactorProgram2, 0);
}
static void _reactorStep3(void) {
    printf("reactor step 3\r\n");
    spFullPWMManager->setProgram(kTop, cReactorProgram3, 3);
    spFullPWMManager->setProgram(kRow1c3, cReactorProgram3, 3);
    spFullPWMManager->setProgram(kRow2c3, cReactorProgram3, 3);
    spFullPWMManager->setProgram(kRow3c3, cReactorProgram3, 3);
    spFullPWMManager->setProgram(kRow1c2, cReactorProgram3, 2);
    spFullPWMManager->setProgram(kRow2c2, cReactorProgram3, 2);
    spFullPWMManager->setProgram(kRow3c2, cReactorProgram3, 2);
    spFullPWMManager->setProgram(kRow1c1, cReactorProgram3, 1);
    spFullPWMManager->setProgram(kRow3c1, cReactorProgram3, 1);
    spFullPWMManager->setProgram(kRow2c1, cReactorProgram3, 0);
}
static LEDLoadFn spReactorSteps[] = { _reactorStep0, _reactorStep1, _reactorStep2, _reactorStep3, nullptr};


// Sonic transducer steps
//-------------------------------------
static void _sonicTransducerStep0(void) {
    printf("sonic transducer step 0\r\n");
    _allOff();
    spFullPWMManager->setProgram(kRow3c1, cDimPulseProgram, 1);
    spFullPWMManager->setProgram(kRow3c2, cDimPulseProgram, 0);
}

int64_t _intoSDLevel1(alarm_id_t id, void *user_data) {
    spFullPWMManager->setProgram(kTop, cSTProgram1, 3);
    spFullPWMManager->setProgram(kRow1c1, cSTProgram1, 2);
    spFullPWMManager->setProgram(kRow1c2, cSTProgram1, 2);
    spFullPWMManager->setProgram(kRow1c3, cSTProgram1, 2);
    spFullPWMManager->setProgram(kRow2c1, cSTProgram1, 1);
    spFullPWMManager->setProgram(kRow2c2, cSTProgram1, 1);
    spFullPWMManager->setProgram(kRow2c3, cSTProgram1, 1);
    spFullPWMManager->setProgram(kRow3c1, cSTProgram1, 0);
    spFullPWMManager->setProgram(kRow3c2, cSTProgram1, 0);
    spFullPWMManager->setProgram(kRow3c3, cSTProgram1, 0);
    return 0;
}

static void _sonicTransducerStep1(void) {
    printf("sonic transducer step 1\r\n");
    spFullPWMManager->setProgram(kTop, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow1c1, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow1c2, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow1c3, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow2c1, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow2c2, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow2c3, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow3c1, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow3c2, cIndicatorsProgram, 0, false);
    spFullPWMManager->setProgram(kRow3c3, cIndicatorsProgram, 0, false);
    add_alarm_in_ms(2000, _intoSDLevel1, nullptr, false);
}

static void _sonicTransducerStep2(void) {
    printf("sonic transducer step 2\r\n");
    spFullPWMManager->setProgram(kTop, cSTProgram2, 3);
    spFullPWMManager->setProgram(kRow1c1, cSTProgram2, 2);
    spFullPWMManager->setProgram(kRow1c2, cSTProgram2, 2);
    spFullPWMManager->setProgram(kRow1c3, cSTProgram2, 2);
    spFullPWMManager->setProgram(kRow2c1, cSTProgram2, 1);
    spFullPWMManager->setProgram(kRow2c2, cSTProgram2, 1);
    spFullPWMManager->setProgram(kRow2c3, cSTProgram2, 1);
    spFullPWMManager->setProgram(kRow3c1, cSTProgram2, 0);
    spFullPWMManager->setProgram(kRow3c2, cSTProgram2, 0);
    spFullPWMManager->setProgram(kRow3c3, cSTProgram2, 0);
}

static void _sonicTransducerStep3(void) {
    printf("sonic transducer step 3\r\n");
    spFullPWMManager->setProgram(kTop, cSTProgram3, 3);
    spFullPWMManager->setProgram(kRow1c1, cSTProgram3, 2);
    spFullPWMManager->setProgram(kRow1c2, cSTProgram3, 2);
    spFullPWMManager->setProgram(kRow1c3, cSTProgram3, 2);
    spFullPWMManager->setProgram(kRow2c1, cSTProgram3, 1);
    spFullPWMManager->setProgram(kRow2c2, cSTProgram3, 1);
    spFullPWMManager->setProgram(kRow2c3, cSTProgram3, 1);
    spFullPWMManager->setProgram(kRow3c1, cSTProgram3, 0);
    spFullPWMManager->setProgram(kRow3c2, cSTProgram3, 0);
    spFullPWMManager->setProgram(kRow3c3, cSTProgram3, 0);
}

static LEDLoadFn spSonicTransducerSteps[] = { _sonicTransducerStep0, _sonicTransducerStep1, _sonicTransducerStep2, _sonicTransducerStep3, nullptr};


// Transit beam/crystal
//-------------------------------------
static void _transitBeamStep0(void) {
    printf("Transit beam step 0\r\n");
    _allOff();
    spFullPWMManager->setProgram(kRow3c1, cDimPulseProgram, 2);
    spFullPWMManager->setProgram(kRow3c2, cDimPulseProgram, 1);
    spFullPWMManager->setProgram(kRow3c3, cDimPulseProgram, 0);
}
static void _transitBeamStep1(void) {
    printf("Transit beam step 1\r\n");
    spFullPWMManager->setProgram(kTop, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow1c1, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow1c2, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow1c3, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow2c1, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow2c2, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow2c3, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow3c1, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow3c2, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow3c3, cTBProgram12, 0);
}
static void _transitBeamStep2(void) {
    printf("Transit beam step 2\r\n");
    spFullPWMManager->setProgram(kTop, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow1c3, cTBProgram12, 0);
    spFullPWMManager->setProgram(kRow1c2, cTBProgram12, 1);
    spFullPWMManager->setProgram(kRow1c1, cTBProgram12, 1);
    spFullPWMManager->setProgram(kRow2c3, cTBProgram12, 1);
    spFullPWMManager->setProgram(kRow2c2, cTBProgram12, 2);
    spFullPWMManager->setProgram(kRow2c1, cTBProgram12, 2);
    spFullPWMManager->setProgram(kRow3c3, cTBProgram12, 2);
    spFullPWMManager->setProgram(kRow3c2, cTBProgram12, 3);
    spFullPWMManager->setProgram(kRow3c1, cTBProgram12, 3);
}
static void _transitBeamStep3(void) {
    printf("Transit beam step 3\r\n");
    spFullPWMManager->setProgram(kTop, cTBProgram3, 0);
    spFullPWMManager->setProgram(kRow1c3, cTBProgram3, 0);
    spFullPWMManager->setProgram(kRow1c2, cTBProgram3, 1);
    spFullPWMManager->setProgram(kRow1c1, cTBProgram3, 1);
    spFullPWMManager->setProgram(kRow2c3, cTBProgram3, 1);
    spFullPWMManager->setProgram(kRow2c2, cTBProgram3, 2);
    spFullPWMManager->setProgram(kRow2c1, cTBProgram3, 2);
    spFullPWMManager->setProgram(kRow3c3, cTBProgram3, 2);
    spFullPWMManager->setProgram(kRow3c2, cTBProgram3, 3);
    spFullPWMManager->setProgram(kRow3c1, cTBProgram3, 3);
}

static LEDLoadFn spTransitBeamSteps[] = { _transitBeamStep0, _transitBeamStep1, _transitBeamStep2, _transitBeamStep3, nullptr};

// Testing 
//-------------------------------------
static void _testingStep0(void) {
    printf("Testing step 0\r\n");
    spFullPWMManager->setProgram(kTop, cLightsTestProgram, 9);
    spFullPWMManager->setProgram(kRow1c1, cLightsTestProgram, 8);
    spFullPWMManager->setProgram(kRow1c2, cLightsTestProgram, 7);
    spFullPWMManager->setProgram(kRow1c3, cLightsTestProgram, 6);
    spFullPWMManager->setProgram(kRow2c1, cLightsTestProgram, 5);
    spFullPWMManager->setProgram(kRow2c2, cLightsTestProgram, 4);
    spFullPWMManager->setProgram(kRow2c3, cLightsTestProgram, 3);
    spFullPWMManager->setProgram(kRow3c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow3c2, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow3c3, cLightsTestProgram, 0);
}
static void _testingStep1(void) {
    printf("Testing step 1\r\n");
    spFullPWMManager->setProgram(kTop, cLightsTestProgram, 3);
    spFullPWMManager->setProgram(kRow1c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow1c2, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow1c3, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow2c1, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow2c2, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow2c3, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow3c1, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow3c2, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow3c3, cLightsTestProgram, 0);
}
static void _testingStep2(void) {
    printf("Testing step 2\r\n");
    spFullPWMManager->setProgram(kTop, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow1c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow1c2, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow1c3, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow2c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow2c2, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow2c3, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow3c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow3c2, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow3c3, cLightsTestProgram, 0);
}
static void _testingStep3(void) {
    printf("Testing step 3\r\n");
    spFullPWMManager->setProgram(kTop, cLightsTestProgram, 1);
    spFullPWMManager->setProgram(kRow1c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow1c2, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow1c3, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow2c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow2c2, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow2c3, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow3c1, cLightsTestProgram, 2);
    spFullPWMManager->setProgram(kRow3c2, cLightsTestProgram, 0);
    spFullPWMManager->setProgram(kRow3c3, cLightsTestProgram, 2);
}
static LEDLoadFn spTestingSteps[] = { _testingStep0, _testingStep1, _testingStep2, _testingStep3};


// Sequence selection logic
//-----------------------------------------------------------------------------
#define kSequenceSteps      (_arraysize(spTestingSteps))
#define kSequences          (4)
static LEDLoadFn *spActiveSequence = spReactorSteps;
static uint suSequenceStep = 0;
static void _resetSequence(void) {
    printf("_resetSequence\r\n");
    suSequenceStep = 0;
    if (spActiveSequence != nullptr) {
        (*spActiveSequence[0])();
    }
}

static void _selectSequenceUp(void) {
    printf("_selectSequenceUp\r\n");
    if (spActiveSequence == spReactorSteps) {
        spActiveSequence = spSonicTransducerSteps;
    } else if (spActiveSequence == spSonicTransducerSteps) {
        spActiveSequence = spTransitBeamSteps;
    } else if (spActiveSequence == spTransitBeamSteps) {
        spActiveSequence = spTestingSteps;
    }
    _resetSequence();
}
static void _selectSequenceDown(void) {
    printf("_selectSequenceDown\r\n");
    if (spActiveSequence == spSonicTransducerSteps) {
        spActiveSequence = spReactorSteps;
    } else if (spActiveSequence == spTransitBeamSteps) {
        spActiveSequence = spSonicTransducerSteps;
    } else if (spActiveSequence == spTestingSteps) {
        spActiveSequence = spTransitBeamSteps;
    }
    _resetSequence();
}
static void _sequenceStepUp(void) {
    printf("_sequenceStepUp\r\n");
    suSequenceStep = (suSequenceStep < (kSequenceSteps-1))?(suSequenceStep+1):suSequenceStep;
   
    if (spActiveSequence != nullptr) {
        (*spActiveSequence[suSequenceStep])();
    }
}
static void _sequenceStepDown(void) {
    printf("_sequenceStepDown\r\n");
    suSequenceStep = (suSequenceStep > 0)?(suSequenceStep-1):suSequenceStep;
    if (spActiveSequence != nullptr) {
        (*spActiveSequence[suSequenceStep])();
    }
}


// Button state and handlers
//-----------------------------------------------------------------------------
static bool sbModeSelectActive = false;
static bool sbInhibitRelease = false;
static void _greenClick(void) {
    if (true == sbModeSelectActive) {
        _selectSequenceUp();
    }
}

static void _greenDoubleClick(void) {
    printf("Green double click\r\n");
}

static void _greenHold(void) {
    //printf("Green HOLD\r\n");
    _resetSequence();
    sbInhibitRelease = true;
}

static void _greenRelease(void) {
    printf("Green release\r\n");
    if ((false == sbInhibitRelease) && (false == sbModeSelectActive)) {
        _sequenceStepUp();
    }
    sbInhibitRelease = false;
}

static void _blueClick(void) {
    if (true == sbModeSelectActive) {
        _selectSequenceDown();
    }
}

static void _blueDoubleClick(void) {
    printf("Blue double click\r\n");
}

static void _blueHold(void) {
    //printf("Blue HOLD\r\n");
    _resetSequence();
    sbInhibitRelease = true;
}

static void _blueRelease(void) {
    printf("Blue release\r\n");
    if ((false == sbInhibitRelease) && (false == sbModeSelectActive)) {
        _sequenceStepDown();
    }
    sbInhibitRelease = false;
}

static void _fobClick(void) {
    //printf("Fob click\r\n");
}

static void _fobDoubleClick(void) {
    //printf("Fob double click\r\n");
}

static void _fobHold(void) {
    //printf("Fob HOLD\r\n");
    _resetSequence();
    sbInhibitRelease = true;
}

static void _fobRelease(void) {
    printf("Fob release\r\n");
    if ((false == sbInhibitRelease) && (false == sbModeSelectActive)) {
        _sequenceStepUp();
    }
    sbInhibitRelease = false;
}

static void _modeSwitchOn(void) {
    printf("Mode switch ON\r\n");
    sbModeSelectActive = true;
}

static void _modeSwitchOff(void) {
    printf("Mode switch OFF\r\n");
    sbModeSelectActive = false;
}

// Button state machine
//-----------------------------------------------------------------------------
static bool _buttonCallback(struct repeating_timer *pTimer) {
    static uint uLastEventTicks(0);
    static bool sbLastGreenState(false), sbCurrentGreenState(false), sbGreenHoldFired(false), sbGreenDown(false);
    static uint suLastGreenDownTime(0);
    static bool sbLastBlueState(false), sbCurrentBlueState(false), sbBlueHoldFired(false), sbBlueDown(false);
    static uint suLastBlueDownTime(0);
    static bool sbLastFobState(false), sbCurrentFobState(false), sbFobHoldFired(false), sbFobDown(false);
    static uint suLastFobDownTime(0);

    static bool sbLastModeSwitchState(false), sbCurrentModeSwitchState(false);

    uint uNow(time_us_32());
    bool bGreenButtonState(!gpio_get(kGreenButton));     // pressed is 'true'
    if (bGreenButtonState == sbLastGreenState) {         // Detected consistent changed state
        if (sbCurrentGreenState != sbLastGreenState) {   // Detected change
            if (false == sbCurrentGreenState) {
                uint uDelay(uNow - suLastGreenDownTime);
                suLastGreenDownTime = uNow;
                //printf("Green uDelay = %dusec\r\n",uDelay);
                sbGreenDown = true;
                if (uDelay < kDoublePressUsec) {
                    _greenDoubleClick();
                } else {
                    _greenClick();
                }
            } else {
                _greenRelease();
                sbGreenDown = sbGreenHoldFired = false;
            }
            sbCurrentGreenState = sbLastGreenState;
        }
    }
    if ((false == sbGreenHoldFired) && (true == sbGreenDown) && ((uNow-suLastGreenDownTime) > kHoldTime)) {
        _greenHold();
        sbGreenHoldFired = true;
    }
    sbLastGreenState = bGreenButtonState;
    
    bool bBlueButtonState(!gpio_get(kBlueButton));  // pressed is 'true'
    if (bBlueButtonState == sbLastBlueState) {         // Detected consistent changed state
        if (sbCurrentBlueState != sbLastBlueState) {   // Detected change
            if (false == sbCurrentBlueState) {
                uint uDelay(uNow - suLastBlueDownTime);
                suLastBlueDownTime = uNow;
                //printf("Blue uDelay = %dusec\r\n",uDelay);
                sbBlueDown = true;
                if (uDelay < kDoublePressUsec) {
                    _blueDoubleClick();
                } else {
                    _blueClick();
                }
           } else {
                _blueRelease();
                sbBlueHoldFired = sbBlueDown = false;
            }
            sbCurrentBlueState = sbLastBlueState;
        }
    }
    if ((false == sbBlueHoldFired) && (true == sbBlueDown) && ((uNow-suLastBlueDownTime) > kHoldTime)) {
        _blueHold();
        sbBlueHoldFired = true;
    }
    sbLastBlueState = bBlueButtonState;

    bool bFobButtonState(!gpio_get(kFobSwitch));  // pressed is 'true'
    if (bFobButtonState == sbLastFobState) {         // Detected consistent changed state
        if (sbCurrentFobState != sbLastFobState) {   // Detected change
            if (false == sbCurrentFobState) {
                uint uDelay(uNow - suLastFobDownTime);
                suLastFobDownTime = uNow;
                //printf("Fob uDelay = %dusec\r\n",uDelay);
                sbFobDown = true;
                if (uDelay < kDoublePressUsec) {
                    _fobDoubleClick();
                } else {
                    _fobClick();
                }
           } else {
                _fobRelease();
                sbFobHoldFired = sbFobDown = false;
            }
            sbCurrentFobState = sbLastFobState;
        }
    }
    if ((false == sbFobHoldFired) && (true == sbFobDown) && ((uNow-suLastFobDownTime) > kHoldTime)) {
        _fobHold();
        sbFobHoldFired = true;
    }
    sbLastFobState = bFobButtonState;

    bool bModeSwitchState(!gpio_get(kModeSwitch));
    if (bModeSwitchState == sbLastModeSwitchState) {         // Detected steady
        if (sbCurrentModeSwitchState != sbLastModeSwitchState) {   // Detected change
            if (false == sbCurrentModeSwitchState) {
                _modeSwitchOn();
            } else {
                _modeSwitchOff();
            }
            sbCurrentModeSwitchState = sbLastModeSwitchState;
        }
    }
    sbLastModeSwitchState = bModeSwitchState;

    return true;
}


// Main program sequence
//-----------------------------------------------------------------------------
int main() {
    // Initialize stdio functionality
    stdio_init_all();

    //busy_wait_ms(10000);
    //printf("Starting up...\r\n");

    const uint pPWMPins[] = {kTop, kRow1c1, kRow1c2, kRow1c3, kRow2c1, kRow2c2, kRow2c3, kRow3c1, kRow3c2, kRow3c3};
    PWMManager cPWMManager(pPWMPins, _arraysize(pPWMPins));
    spPWMManager = spFullPWMManager = &cPWMManager;    // Link for ISR and button callback routines
    
    // Set it running
    _resetSequence();

    // Set up button and switch GPIOs and add timer for button/switch state detection
    gpio_init(kBlueButton);
    gpio_set_input_enabled(kBlueButton, true);
    gpio_pull_up(kBlueButton);
    gpio_set_input_hysteresis_enabled(kBlueButton, true);

    gpio_init(kGreenButton);
    gpio_set_input_enabled(kGreenButton, true);
    gpio_pull_up(kGreenButton);
    gpio_set_input_hysteresis_enabled(kGreenButton, true);

    gpio_init(kModeSwitch);
    gpio_set_input_enabled(kModeSwitch, true);
    gpio_pull_up(kModeSwitch);
    gpio_set_input_hysteresis_enabled(kModeSwitch, true); 

    gpio_init(kFobSwitch);
    gpio_set_input_enabled(kFobSwitch, true);
    gpio_pull_up(kFobSwitch);
    gpio_set_input_hysteresis_enabled(kFobSwitch, true);

    _modeSwitchOff();   // Force initial state

    struct repeating_timer cButtonTimer;
    add_repeating_timer_ms(10, _buttonCallback, NULL, &cButtonTimer);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (1)
        tight_loop_contents();
}
