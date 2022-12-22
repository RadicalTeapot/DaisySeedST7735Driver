#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

#define NOP 0x00
#define SWRESET 0x01
#define RDDID 0x04
#define RDDST 0x09

#define SLPIN 0x10
#define SLPOUT 0x11
#define PTLON 0x12
#define NORON 0x13

#define INVOFF 0x20
#define INVON 0x21
#define DISPOFF 0x28
#define DISPON 0x29
#define RAMRD 0x2E
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C

#define PTLAR 0x30
#define MADCTL 0x36
#define COLMOD 0x3A

#define FRMCTR1 0xB1
#define FRMCTR2 0xB2
#define FRMCTR3 0xB3
#define INVCTR 0xB4
#define DISSET5 0xB6

#define PWCTR1 0xC0
#define PWCTR2 0xC1
#define PWCTR3 0xC2
#define PWCTR4 0xC3
#define PWCTR5 0xC4
#define VMCTR1 0xC5

#define RDID1 0xDA
#define RDID2 0xDB
#define RDID3 0xDC
#define RDID4 0xDD

#define GMCTRP1 0xE0
#define GMCTRN1 0xE1

#define PWCTR6 0xFC

// 565 COLORS
#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define ORANGE 0xFC00

// AREA definition
// -----------------------------------
#define MAX_X 160                      // max rows
#define MAX_Y 128                      // max columns
#define SIZE_X MAX_X - 1               // rows max counter
#define SIZE_Y MAX_Y - 1               // columns max counter
#define CACHE_SIZE_MEM (MAX_X * MAX_Y) // whole pixels
#define CHARS_COLS_LEN 5               // number of columns for chars
#define CHARS_ROWS_LEN 8               // number of rows for chars

// Penta minor notes
#define C 130.81f
#define DS 155.56f
#define F 174.61f
#define G 196.0f
#define AS 233.08f

/**
 * Pin connections
 *
 * SPI1 CS (D7) -> TFT_CS
 * D2 -> D/C
 * D3 -> RESET
 * SPI1 CLK (D8) -> SCLK
 * SPI1 MOSI (D10) -> MOSI
 *
*/

static DaisySeed hw;
static SpiHandle spiHandle;
static GPIO tftCS, dc, reset, backlight;

static Oscillator osc[5];
static AdEnv env;
static Svf svf;
static ReverbSc reverb;
static DelayLine<float, 24000> delay;
static CrossFade reverbMix, delayMix;

const float kDetune[5] = {-3.2f, -1.1f, 0, 1.1f, 3.2f};
const float kLevel[5] = {0.1f, 0.2f, 0.4f, 0.2f, 0.1f};

const float kFilterBaseFreq = 600.0f;
const float kAmp = 0.33f;
const float kDelayFeedback = 0.66f;
const size_t kScopeScale = 3;
float lastOutValue = 0.0f;

void HWReset() {
    reset.Write(true);
    System::Delay(200);
    reset.Write(false);
    System::Delay(200);
    reset.Write(true);
}

void SendCommand(uint8_t cmd) {
    tftCS.Write(false);
    dc.Write(false);    // We're sending a command so set the data / command pin low to mark it as such
    spiHandle.BlockingTransmit(&cmd, 1);
    tftCS.Write(true);
}

void SendData(uint8_t *data, size_t size) {
    tftCS.Write(false);
    dc.Write(true);    // We're sending data so set the data / command pin high to mark it as such
    spiHandle.BlockingTransmit(data, size);
    tftCS.Write(true);
}

// void SendData(uint16_t data) {
//     uint8_t d[2] = {
//         static_cast<uint8_t>((data >> 8) & 0xFF),  // High byte
//         static_cast<uint8_t>(data & 0xFF) // low byte
//     };
//     SendData(d, 2);
// }

void SendCommandAndDataAndDelay(uint8_t cmd, uint8_t *data, size_t dataSize, uint32_t delay)
{
    SendCommand(cmd);
    if (dataSize > 0)
        SendData(data, dataSize);

    if (delay > 0)
        System::Delay(delay);
}

void SetWindow(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1)
{
    // check if coordinates is out of range
    if ((x0 > x1) || (x1 > SIZE_X) || (y0 > y1) || (y1 > SIZE_Y))
    {
        // out of range
        // hw.PrintLine("Out of range");
        return;
    }
    // row address set
    SendCommand(RASET);
    {
        uint8_t data[4] = {
            static_cast<uint8_t>((x0 >> 8) & 0xFF),
            static_cast<uint8_t>(x0 & 0xFF),
            static_cast<uint8_t>((x1 >> 8) & 0xFF),
            static_cast<uint8_t>(x1 & 0xFF),
        };
        SendData(data, 4);
    }

    // columns address set
    SendCommand(CASET);
    {
        uint8_t data[4] = {
            static_cast<uint8_t>((y0 >> 8) & 0xFF),
            static_cast<uint8_t>(y0 & 0xFF),
            static_cast<uint8_t>((y1 >> 8) & 0xFF),
            static_cast<uint8_t>(y1 & 0xFF),
        };
        SendData(data, 4);
    }
}

void SendColor(uint16_t color, size_t count)
{
    SendCommand(RAMWR);

    //while (count--)
    //    SendData(color);

    // TODO test this, it should be more efficient than the commented code above
    uint8_t colorHigh = static_cast<uint8_t>((color >> 8) & 0xFF);  // High byte
    uint8_t colorLow = static_cast<uint8_t>(color & 0xFF);          // Low byte

    size_t dataSize = 2 * count;
    uint8_t data[dataSize];
    for (size_t i = 0; i < count; i++) {
        data[i] = colorHigh;
        data[i+1] = colorLow;
    }
    SendData(data, dataSize);
}

void ClearScreen(uint16_t color)
{
    SetWindow(0, SIZE_X, 0, SIZE_Y);
    SendColor(color, CACHE_SIZE_MEM);
}

void DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    SetWindow(x, x, y, y);
    SendColor(color, 1);
}

void FillRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    SetWindow(x1, x2, y1, y2);
    SendColor(color, (x2-x1+1) * (y2-y1+1));
}

void DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    if (y1 == y2)
    {
        for (uint x = x1; x <= x2; x++)
            DrawPixel(x, y1, color);
        return;
    }

    // DDA line algo
    float dx = abs(x2 - x1);
    float dy = abs(y2 - y1);
    float signY = y2 > y1 ? 1 : -1;
    float steps = (float)(dx >= dy ? dx : dy);
    dx /= steps; dy = (dy/steps) * signY;
    float x = x1, y = y1;
    for (uint i = 0 ; i <= steps; i++)
    {
        DrawPixel(x, y, WHITE);
        x += dx; y += dy;
    }
}

void Init() {
    // Software reset
    SendCommandAndDataAndDelay(SWRESET, 0, 0, 150);
    // Out of sleep mode
    SendCommandAndDataAndDelay(SLPOUT, 0, 0, 500);

    // Set framerate control
    uint8_t args[6];
    args[0] = 0x01; args[1] = 0x2C; args[2] = 0x2D;     // 333k / ((1 + 20) * (LINE + 44 + 45))
    args[3] = 0x01; args[4] = 0x2C; args[5] = 0x2D;
    SendCommandAndDataAndDelay(FRMCTR1, args, 3, 0);
    SendCommandAndDataAndDelay(FRMCTR2, args, 3, 0);
    SendCommandAndDataAndDelay(FRMCTR3, args, 6, 0);

    // Set inversion control to no inversion
    args[0] = 0x07;
    SendCommandAndDataAndDelay(INVCTR, args, 1, 0);

    // Power control
    args[0] = 0xA2; args[1] = 0x02; args[2] = 0x84;
    SendCommandAndDataAndDelay(PWCTR1, args, 3, 0);
    args[0] = 0xC5;
    SendCommandAndDataAndDelay(PWCTR2, args, 1, 0);
    args[0] = 0x0A; args[1] = 0x00;
    SendCommandAndDataAndDelay(PWCTR3, args, 2, 0);
    args[0] = 0x8A; args[1] = 0x2A;
    SendCommandAndDataAndDelay(PWCTR4, args, 2, 0);
    args[0] = 0x8A; args[1] = 0xEE;
    SendCommandAndDataAndDelay(PWCTR5, args, 2, 0);
    args[0] = 0x0E;
    SendCommandAndDataAndDelay(VMCTR1, args, 1, 0);

    // Turn off inversion
    SendCommandAndDataAndDelay(INVOFF, 0, 0, 0);
    // Set coordinate system (row / col address, bottom-top refresh)
    args[0] = 0xC8;
    SendCommandAndDataAndDelay(MADCTL, args, 1, 0);
    // Set color mode to 16 bit per pixels
    args[0] = 0x05;
    SendCommandAndDataAndDelay(COLMOD, args, 1, 10);
    // Send row address set
    args[0] = 0; args[1] = (uint8_t) SIZE_X;
    SendCommandAndDataAndDelay(RASET, args, 2, 0);
    // Send column address set
    args[0] = 0; args[1] = (uint8_t) SIZE_Y;
    SendCommandAndDataAndDelay(CASET, args, 2, 0);
    // Send normal display on
    SendCommandAndDataAndDelay(NORON, 0, 0, 10);
    // Main screen turn on
    SendCommandAndDataAndDelay(DISPON, 0, 0, 100);
}

void AudioCallback( AudioHandle::InterleavingInputBuffer in,
                    AudioHandle::InterleavingOutputBuffer out,
                    size_t size)
{
    float oscValue, envValue, outValue, delayValue, delayOut;
    float reverbOut[2], mixOut[2];
    size_t bufferHeadInt;
    for (size_t i = 0; i < size; i+=2) {
        oscValue = 0;
        for (size_t j = 0; j < 5; j++)
            oscValue += osc[j].Process();
        svf.Process(oscValue);

        envValue = env.Process();
        outValue = svf.Low() * envValue;

        delay.Write(outValue + delay.Read() * kDelayFeedback);
        delayValue = delay.Read();
        delayOut = delayMix.Process(outValue, delayValue);

        reverb.Process(delayOut, delayOut, reverbOut, (reverbOut+1));
        mixOut[0] = reverbMix.Process(delayOut, reverbOut[0]);
        mixOut[1] = reverbMix.Process(delayOut, reverbOut[1]);

        out[i] = mixOut[0] * kAmp;
        out[i+1] = mixOut[1] * kAmp;

        lastOutValue = mixOut[0] + (mixOut[1] - mixOut[0]) * 0.5f;
    }
}

int main(void)
{
    // Initialize seed hardware
    hw.Configure();
    hw.Init();

    hw.SetAudioBlockSize(4);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    float sampleRate = hw.AudioSampleRate();
    for (size_t i = 0; i < 5; i++)
    {
        osc[i].Init(sampleRate);
        osc[i].SetFreq(C + kDetune[i]);
        osc[i].SetWaveform(Oscillator::WAVE_TRI);
        osc[i].SetAmp(kLevel[i]);
    }
    env.Init(sampleRate);
    env.SetTime(ADENV_SEG_ATTACK, 0.05f);

    svf.Init(sampleRate);
    svf.SetRes(0.5f);
    svf.SetFreq(kFilterBaseFreq);

    delay.Init();
    delay.SetDelay(sampleRate * 0.25f);

    delayMix.Init(CROSSFADE_CPOW);
    delayMix.SetPos(0.3f);

    reverb.Init(sampleRate);
    reverb.SetFeedback(0.95f);
    reverb.SetLpFreq(AS * 1.5f);

    reverbMix.Init(CROSSFADE_CPOW);
    reverbMix.SetPos(0.25f);

    SpiHandle::Config spiConf;
    spiConf.periph = SpiHandle::Config::Peripheral::SPI_1;                  // Using SPI1 pins
    spiConf.mode = SpiHandle::Config::Mode::MASTER;                         // Daisy seed is master
    spiConf.direction = SpiHandle::Config::Direction::TWO_LINES_TX_ONLY;    // Daisy seed MOSI -> out
    spiConf.nss = SpiHandle::Config::NSS::SOFT;
    spiConf.pin_config.sclk = Pin(GPIOPort::PORTG, 11);                     // PG11 = D8 (from datasheet)
    spiConf.pin_config.mosi = Pin(GPIOPort::PORTB, 5);                      // PB5 = D10 (from datasheet)
    spiConf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_2;        // 25 / 2 = 12.5MHz
    spiConf.datasize = 8;                                                   // Send 8 bit packets

    // Init pins
    tftCS.Init(daisy::seed::D7, GPIO::Mode::OUTPUT);
    dc.Init(daisy::seed::D2, GPIO::Mode::OUTPUT);
    reset.Init(daisy::seed::D3, GPIO::Mode::OUTPUT);
    backlight.Init(daisy::seed::D4, GPIO::Mode::OUTPUT);

    spiHandle.Init(spiConf);

    tftCS.Write(true);
    backlight.Write(true);

    // Hardware reset
    HWReset();
    // Init commands
    Init();
    // Clear screen to black
    ClearScreen(BLACK);

    hw.StartAudio(AudioCallback);

    const size_t delayTime = 10;
    const float noteLength[4] = {200, 400, 800, 1600};
    float nextNoteLength = noteLength[0];
    size_t lastNoteElapsedTime = nextNoteLength;
    const float noteAmp[4] = {0.0f, 0.5f, 0.75f, 1.0f};
    const float notes[5] = {C, DS, F, G, AS};
    float nextFreq;
    float detuneDrift[5] = {1, 1, 1, 1, 1};

    size_t position = 0;
    uint8_t lineEndY = 0, lastLineEndY = 0;
	for (;;)
	{
        FillRect(position, 0, position + kScopeScale, SIZE_Y, BLACK);
        lineEndY = static_cast<uint8_t>(fmap(lastOutValue * 0.5f + 0.5f, 0, SIZE_Y));
        DrawLine(position, lastLineEndY, position + kScopeScale, lineEndY, WHITE);
        lastLineEndY = lineEndY;
        position += kScopeScale;
        if (position >= SIZE_X - kScopeScale) position = 0;

        if (lastNoteElapsedTime >= nextNoteLength)
        {
            nextFreq = notes[static_cast<size_t>(fmap(rand() * kRandFrac, 0, 5))];
            nextNoteLength = noteLength[static_cast<size_t>(fmap(rand() * kRandFrac, 0, 4))];
            for (size_t i = 0; i < 5; i++)
            {
                fonepole(detuneDrift[i], fmap(rand() * kRandFrac, 0.5f, 2.0f), 0.1f);
                osc[i].SetFreq(nextFreq + kDetune[i] * detuneDrift[i]);
            }
            svf.SetFreq(kFilterBaseFreq + nextFreq * 0.25f);
            env.SetTime(ADENV_SEG_DECAY, nextNoteLength * 0.1f * 1e-3f);
            env.SetMax(noteAmp[static_cast<size_t>(fmap(rand() * kRandFrac, 0, 4))]);
            env.Trigger();
            lastNoteElapsedTime = 0;
        }
        lastNoteElapsedTime += delayTime;
        hw.DelayMs(delayTime);
	}
}
