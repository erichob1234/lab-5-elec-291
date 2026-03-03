// ===================== NON-BLOCKING INSTRUMENT VERSION =====================
// Fixes the “blank LCD / stuck in while() waits” by eliminating ALL blocking
// edge-waits. Uses a free-running Timer0 timestamp + edge-detect state machines.
//
// Modes:
// 0: V/I RMS + Phase
// 1: Gain + dB (approx)
// 2: Power: P (top), Q (bottom)  [small-angle trig approx]
// 3: Placeholder
//
// Pins (final):
//   P0.1 = REF square wave
//   P0.2 = TEST square wave
//   P2.6 = REF analog (Voltage)
//   P2.5 = TEST analog (Current sensor output scaled to "mA as mV")
//   P2.2 = Button (active-low), hold ~1–2s ok
//   P2.1 = LED (active-high): ON when frequency mismatch
//
// Notes:
// - No blocking while(pin...) anywhere.
// - Timer0 runs free, ISR extends to 32-bit timebase.
// - Rising edges detected by sampling pin each loop.
// - Periods from consecutive rising edge timestamps.
// - Phase from most recent REF and TEST rising edges (wrap to [-T/2, +T/2]).
//   Sign convention matches your earlier code: TEST lag => negative phase.
// - RMS sampled at (edge + T/4) using a scheduled timestamp (no wait loops).
// - Frequency mismatch LED uses hysteresis (same idea as before).

#include <EFM8LB1.h>

#define SYSCLK  72000000L
#define SARCLK  18000000L

// LCD pins
#define LCD_RS P1_7
#define LCD_E  P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0

// UI pins
#define MODE_BTN      P2_2
#define MISMATCH_LED  P2_1

// Signal pins
#define REF_SQ   P0_1
#define TEST_SQ  P0_2

// ADC constants
#define VDD_UV 3303500UL
#define ADC_FS 16383UL

// Fixed-point
#define RMS_NUM 7071UL
#define RMS_DEN 10000UL

// Timer0 tick rate in Mode1 on this setup: SYSCLK/12
#define T0_TICKS_PER_SEC (SYSCLK/12UL)

// ===== Small DATA (keep tiny) =====
unsigned char overflow_count;   // kept for compatibility (not used for period now)
unsigned char mode = 0;
bit last_btn = 1;
bit mismatch = 0;

// ===== 32-bit free-running timebase =====
volatile xdata unsigned long t0_hi = 0;  // increments every Timer0 overflow (65536 ticks)

// ===== Measurement state (xdata to avoid DATA overflow) =====
xdata unsigned long t_ref_last;
xdata unsigned long t_ref_prev;
xdata unsigned long t_test_last;
xdata unsigned long t_test_prev;

xdata unsigned long period_ref;
xdata unsigned long period_test;

xdata bit ref_valid;
xdata bit test_valid;

// Edge detect previous pin states
xdata bit ref_prev_state;
xdata bit test_prev_state;

// Scheduled RMS sampling
xdata unsigned long ref_sample_time;
xdata unsigned long test_sample_time;
xdata bit ref_sample_pending;
xdata bit test_sample_pending;

xdata unsigned int ref_amp_mV;
xdata unsigned int test_amp_mV;
xdata unsigned int ref_rms_mV;
xdata unsigned int test_rms_mV;

// Phase in centi-degrees
xdata long phase_cdeg;

// Gain/dB
xdata unsigned long gain_x10000;
xdata unsigned long ratio_q16_16;
xdata long db_centi;

// Power
xdata long VI_mW;
xdata long phi_rad_q15;
xdata long sinphi_q15;
xdata long cosphi_q15;
xdata long phi2_q15;
xdata long real_power_mW;
xdata long reactive_power_mW;

// LCD buffers
xdata char line1[17];
xdata char line2[17];

// ===================== STARTUP =====================
char _c51_external_startup(void)
{
	SFRPAGE=0x00;
	WDTCN=0xDE;
	WDTCN=0xAD;

	VDM0CN=0x80;
	RSTSRC=0x02|0x04;

	// SYSCLK = 72 MHz
	SFRPAGE=0x10;
	PFE0CN=0x20;
	SFRPAGE=0x00;

	CLKSEL=0x00; CLKSEL=0x00; while(!(CLKSEL&0x80));
	CLKSEL=0x03; CLKSEL=0x03; while(!(CLKSEL&0x80));

	XBR2=0x40; // crossbar + weak pullups

	// Ensure P0.1/P0.2 are digital inputs
	SFRPAGE=0x20;
	P0MDIN |= 0x06; // P0.1, P0.2 digital
	SFRPAGE=0x00;

	return 0;
}

// ===================== TIMER0 FREE-RUNNING + ISR =====================
void TIMER0_Init_FreeRun(void)
{
	// Timer0 Mode 1, 16-bit
	TMOD &= 0xF0;
	TMOD |= 0x01;

	TR0 = 0;
	TL0 = 0;
	TH0 = 0;
	TF0 = 0;

	ET0 = 1;  // enable Timer0 interrupt
	EA  = 1;  // global enable

	TR0 = 1;  // start free-running
}

// Timer0 overflow ISR: extend to 32-bit time
void Timer0_ISR(void) interrupt 1
{
	TF0 = 0;
	t0_hi++;
}

// Atomic read of 32-bit timestamp = (t0_hi<<16) | (TH0:TL0)
unsigned long time_now_ticks(void)
{
	unsigned long hi1, hi2;
	unsigned char th, tl;
	unsigned long t;

	EA = 0;
	hi1 = t0_hi;
	th  = TH0;
	tl  = TL0;
	hi2 = t0_hi;
	EA = 1;

	// If overflow happened during read, re-read quickly
	if(hi2 != hi1)
	{
		EA = 0;
		hi1 = t0_hi;
		th  = TH0;
		tl  = TL0;
		EA = 1;
	}

	t = (hi1<<16) | ((unsigned long)th<<8) | tl;
	return t;
}

// ===================== TIMER3 delay (LCD) =====================
void Timer3us(unsigned char us)
{
	unsigned char i;
	CKCON0 |= 0x40;
	TMR3RL = -(SYSCLK/1000000L);
	TMR3   = TMR3RL;
	TMR3CN0 = 0x04;
	for(i=0;i<us;i++)
	{
		while(!(TMR3CN0 & 0x80));
		TMR3CN0 &= ~0x80;
	}
	TMR3CN0 = 0;
}

void waitms(unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0;j<ms;j++)
		for(k=0;k<4;k++) Timer3us(250);
}

// ===================== LCD =====================
void LCD_pulse(void){ LCD_E=1; Timer3us(40); LCD_E=0; }

void LCD_byte(unsigned char x)
{
	ACC=x;
	LCD_D7=ACC_7; LCD_D6=ACC_6; LCD_D5=ACC_5; LCD_D4=ACC_4;
	LCD_pulse(); Timer3us(40);
	ACC=x;
	LCD_D7=ACC_3; LCD_D6=ACC_2; LCD_D5=ACC_1; LCD_D4=ACC_0;
	LCD_pulse();
}

void LCD_cmd(unsigned char x){ LCD_RS=0; LCD_byte(x); waitms(2); }
void LCD_data(unsigned char x){ LCD_RS=1; LCD_byte(x); waitms(1); }

void LCD_init(void)
{
	LCD_E=0;
	waitms(20);
	LCD_cmd(0x33); LCD_cmd(0x33); LCD_cmd(0x32);
	LCD_cmd(0x28); LCD_cmd(0x0C); LCD_cmd(0x01);
	waitms(20);
}

void LCDprint(xdata char *s, unsigned char line)
{
	unsigned char i;
	LCD_cmd(line==2?0xC0:0x80);
	for(i=0;i<16;i++)
	{
		if(s[i]==0) break;
		LCD_data(s[i]);
	}
	for(;i<16;i++) LCD_data(' ');
}

// ===================== ADC =====================
void InitADC(void)
{
	ADEN=0;
	ADC0CN1=(0x2<<6);                 // 14-bit
	ADC0CF0=((SYSCLK/SARCLK)<<3);
	ADC0CF2=(0x1<<5)|(0x1F);          // VDD ref
	ADEN=1;
}

void InitPinADC(unsigned char portno,unsigned char pinno)
{
	unsigned char mask = 1<<pinno;
	SFRPAGE=0x20;
	if(portno==2)
	{
		P2MDIN &= (~mask);
		P2SKIP |= mask;
	}
	SFRPAGE=0x00;
}

unsigned int ADC_read(unsigned char pin)
{
	ADC0MX=pin;
	ADINT=0;
	ADBUSY=1;
	while(!ADINT);
	return ADC0;
}

unsigned int mV_read(unsigned char pin)
{
	unsigned long adc;
	unsigned long uv;
	adc = ADC_read(pin);
	uv  = (adc*VDD_UV)/ADC_FS;
	return (unsigned int)((uv+500)/1000);
}

// ===================== MATH HELPERS =====================
long wrap_phase(long p)
{
	while(p>18000L) p-=36000L;
	while(p<-18000L) p+=36000L;
	return p;
}

// dt/T -> centi-deg
long phase_from_dt(unsigned long dt, unsigned long T)
{
	unsigned long t360, deg, rem;
	if(T==0) return 0;
	t360 = dt*360UL;
	deg  = t360/T;
	rem  = t360%T;
	return (long)(deg*100UL + (rem*100UL)/T);
}

// Q15 radians from centi-deg, (pi/180)*32768 ˜ 572
long phase_to_rad_q15(long cdeg)
{
	return (cdeg * 572L) / 100L;
}

// dB helpers (approx) same as earlier
int log2_q8_8(unsigned long m)
{
	int e;
	unsigned long u,u2,u3;
	long ln;
	long frac;
	long total;

	e=0;
	if(m==0) return -32768;

	while(m>=(2UL<<16)){ m>>=1; e++; }
	while(m<(1UL<<16)) { m<<=1; e--; }

	u  = m-(1UL<<16);
	u2 = (u*u)>>16;
	u3 = (u2*u)>>16;

	ln = (long)u - (long)(u2>>1) + (long)(u3/3UL);

	frac  = (ln*94548L)>>16;
	total = ((long)e<<16) + frac;
	return (int)(total>>8);
}

long db_centi_from_ratio_q16_16(unsigned long r)
{
	int l2;
	l2 = log2_q8_8(r);
	return ((long)l2*60206L)/256L;
}

// ===================== DISPLAY HELPERS =====================
void clear_line(xdata char *s)
{
	unsigned char i;
	for(i=0;i<16;i++) s[i]=' ';
	s[16]=0;
}
void put_char(xdata char *s, unsigned char p, char c){ if(p<16) s[p]=c; }
void put_str(xdata char *s, unsigned char p, const char *t){ while(*t && p<16) s[p++]=*t++; }

void put_mV(xdata char *s, unsigned char p, unsigned int mv)
{
	unsigned int v;
	unsigned int f;
	v = mv/1000;
	f = (mv%1000)/10;
	put_char(s,p+0,'0'+(v%10));
	put_char(s,p+1,'.');
	put_char(s,p+2,'0'+(f/10));
	put_char(s,p+3,'0'+(f%10));
}

void put_centi_signed(xdata char *s, unsigned char p, long value_centi)
{
	char sign;
	unsigned long a;
	unsigned int d;
	unsigned int f;

	sign = '+';
	if(value_centi<0){ sign='-'; a=(unsigned long)(-value_centi); }
	else { a=(unsigned long)value_centi; }

	d = (unsigned int)(a/100UL);
	f = (unsigned int)(a%100UL);

	put_char(s,p+0,sign);
	put_char(s,p+1,'0'+((d/100)%10));
	put_char(s,p+2,'0'+((d/10)%10));
	put_char(s,p+3,'0'+(d%10));
	put_char(s,p+4,'.');
	put_char(s,p+5,'0'+(f/10));
	put_char(s,p+6,'0'+(f%10));
}

void put_gain(xdata char *s, unsigned char p, unsigned long g)
{
	unsigned int i;
	unsigned int f;
	i = (unsigned int)(g/10000UL);
	f = (unsigned int)(g%10000UL);

	put_char(s,p+0,'0'+(i%10));
	put_char(s,p+1,'.');
	put_char(s,p+2,'0'+(f/1000));
	put_char(s,p+3,'0'+((f/100)%10));
	put_char(s,p+4,'0'+((f/10)%10));
	put_char(s,p+5,'0'+(f%10));
}

void put_db(xdata char *s, unsigned char p, long cdb)
{
	char sign;
	unsigned long a;
	unsigned int d;
	unsigned int f;

	sign = '+';
	if(cdb<0){ sign='-'; a=(unsigned long)(-cdb); }
	else { a=(unsigned long)cdb; }

	d = (unsigned int)(a/100UL);
	f = (unsigned int)(a%100UL);

	put_char(s,p+0,sign);
	put_char(s,p+1,'0'+((d/10)%10));
	put_char(s,p+2,'0'+(d%10));
	put_char(s,p+3,'.');
	put_char(s,p+4,'0'+(f/10));
	put_char(s,p+5,'0'+(f%10));
}

// ===================== EDGE + SCHEDULER UPDATE =====================
void update_edges_and_schedule(void)
{
	unsigned long t;
	bit r, s;

	t = time_now_ticks();

	// REF edge detect
	r = REF_SQ;
	if((ref_prev_state==0) && (r==1))
	{
		// rising edge
		t_ref_prev = t_ref_last;
		t_ref_last = t;

		if(t_ref_prev != 0)
		{
			period_ref = t_ref_last - t_ref_prev;
			ref_valid = 1;

			// schedule RMS sample at T/4 after edge
			ref_sample_time = t_ref_last + (period_ref>>2);
			ref_sample_pending = 1;
		}
	}
	ref_prev_state = r;

	// TEST edge detect
	s = TEST_SQ;
	if((test_prev_state==0) && (s==1))
	{
		t_test_prev = t_test_last;
		t_test_last = t;

		if(t_test_prev != 0)
		{
			period_test = t_test_last - t_test_prev;
			test_valid = 1;

			test_sample_time = t_test_last + (period_test>>2);
			test_sample_pending = 1;
		}
	}
	test_prev_state = s;
}

// Non-blocking RMS sampling when scheduled time has passed
void update_rms_samples(void)
{
	unsigned long t;
	t = time_now_ticks();

	if(ref_sample_pending && ref_valid)
	{
		if((long)(t - ref_sample_time) >= 0)
		{
			ref_amp_mV = mV_read(QFP32_MUX_P2_6);
			ref_rms_mV = (unsigned int)(((unsigned long)ref_amp_mV*RMS_NUM)/RMS_DEN);
			ref_sample_pending = 0;
		}
	}

	if(test_sample_pending && test_valid)
	{
		if((long)(t - test_sample_time) >= 0)
		{
			test_amp_mV = mV_read(QFP32_MUX_P2_5);
			test_rms_mV = (unsigned int)(((unsigned long)test_amp_mV*RMS_NUM)/RMS_DEN);
			test_sample_pending = 0;
		}
	}
}

// Compute phase non-blocking using latest timestamps
void update_phase(void)
{
	unsigned long dt;
	long dt_signed;

	if(!ref_valid || !test_valid) { phase_cdeg = 0; return; }
	if(period_ref == 0) { phase_cdeg = 0; return; }

	// dt = test - ref (signed)
	dt_signed = (long)(t_test_last - t_ref_last);

	// wrap dt into [-T/2, +T/2] in ticks
	if(dt_signed > (long)(period_ref>>1))  dt_signed -= (long)period_ref;
	if(dt_signed < -(long)(period_ref>>1)) dt_signed += (long)period_ref;

	// Convert to centi-deg. Sign convention: TEST lags => negative phase.
	// If TEST happens after REF, dt_signed > 0, that is a lag, so phase should be negative.
	if(dt_signed >= 0)
	{
		dt = (unsigned long)dt_signed;
		phase_cdeg = -phase_from_dt(dt, period_ref);
	}
	else
	{
		dt = (unsigned long)(-dt_signed);
		phase_cdeg = phase_from_dt(dt, period_ref);
	}

	phase_cdeg = wrap_phase(phase_cdeg);
}

// Mismatch LED with hysteresis (only if both valid)
void update_mismatch_led(void)
{
	long diff;
	unsigned long ad;

	if(!ref_valid || !test_valid || period_ref==0)
	{
		// If missing signal(s), treat as mismatch (instrument not locked)
		MISMATCH_LED = 1;
		mismatch = 1;
		return;
	}

	diff = (long)period_test - (long)period_ref;
	ad   = (diff<0)? (unsigned long)(-diff) : (unsigned long)diff;

	// ON threshold ~0.25%, OFF threshold ~0.13%
	if(!mismatch)
	{
		if(ad*400UL > period_ref) mismatch = 1;
	}
	else
	{
		if(ad*770UL < period_ref) mismatch = 0;
	}

	MISMATCH_LED = mismatch;
}

// ===================== MAIN =====================
void main(void)
{
	unsigned long t_last_lcd;
	unsigned long tnow;
	unsigned char i;

	// LED push-pull
	SFRPAGE=0x20;
	P2MDOUT |= 0x02; // P2.1 push-pull
	SFRPAGE=0x00;
	MISMATCH_LED = 0;

	// ADC pins
	InitPinADC(2,5);
	InitPinADC(2,6);
	InitADC();

	LCD_init();

	// Timer0 free-running timebase
	TIMER0_Init_FreeRun();

	// init edge states
	ref_prev_state = REF_SQ;
	test_prev_state = TEST_SQ;

	ref_valid = 0;
	test_valid = 0;
	ref_sample_pending = 0;
	test_sample_pending = 0;

	// LCD refresh timer
	t_last_lcd = time_now_ticks();

	while(1)
	{
		// Button edge detect (active-low)
		if((MODE_BTN==0) && (last_btn==1)) mode=(mode+1)&0x03;
		last_btn = MODE_BTN;

		// Update edge timestamps and scheduled sampling
		update_edges_and_schedule();
		update_rms_samples();
		update_phase();
		update_mismatch_led();

		// Compute gain/dB when possible
		if(ref_rms_mV>0)
		{
			gain_x10000  = ((unsigned long)test_rms_mV*10000UL)/ref_rms_mV;
			ratio_q16_16 = ((unsigned long)test_rms_mV<<16)/ref_rms_mV;
			db_centi     = db_centi_from_ratio_q16_16(ratio_q16_16);
		}
		else
		{
			gain_x10000 = 0;
			db_centi = 0;
		}

		// Compute power when possible
		// (Uses current ref_rms_mV/test_rms_mV + phase_cdeg)
		VI_mW = ((long)ref_rms_mV * (long)test_rms_mV) / 1000L;

		phi_rad_q15 = phase_to_rad_q15(phase_cdeg);
		sinphi_q15  = phi_rad_q15;
		phi2_q15    = (phi_rad_q15 * phi_rad_q15) >> 15;
		cosphi_q15  = 32768L - (phi2_q15 >> 1);

		real_power_mW     = (VI_mW * cosphi_q15) >> 15;
		reactive_power_mW = (VI_mW * sinphi_q15) >> 15;

		// LCD refresh at ~5–10 Hz independent of signal presence
		tnow = time_now_ticks();
		if((unsigned long)(tnow - t_last_lcd) > (T0_TICKS_PER_SEC/10UL)) // ~100 ms
		{
			t_last_lcd = tnow;

			clear_line(line1);
			clear_line(line2);

			if(mode==0)
			{
				put_str(line1,0,"V=");
				put_mV(line1,2,ref_rms_mV);
				put_str(line1,7,"I=");
				put_mV(line1,9,test_rms_mV);

				put_str(line2,0,"Ph=");
				put_centi_signed(line2,3,phase_cdeg);
			}
			else if(mode==1)
			{
				put_str(line1,0,"G=");
				put_gain(line1,2,gain_x10000);

				put_str(line2,0,"dB=");
				put_db(line2,3,db_centi);
			}
			else if(mode==2)
			{
				// Display as centi-W: 0.01W = 10mW => centiW = mW/10
				put_str(line1,0,"P=");
				put_centi_signed(line1,2,(real_power_mW/10L));
				put_str(line2,0,"Q=");
				put_centi_signed(line2,2,(reactive_power_mW/10L));
			}
			else
			{
				put_str(line1,0,"MODE 3");
				put_str(line2,0,"(unused)");
			}

			LCDprint(line1,1);
			LCDprint(line2,2);
		}

		// short idle to reduce LCD/ADC hammering (non-blocking measurement still works)
		for(i=0;i<2;i++) Timer3us(200);
	}
}