// ===================== MEMORY-SAFE: MODE0 + MODE1 + MODE2(POWER) =====================
// Keil C51 (C90) friendly, avoids DATA overflow by placing large variables in XDATA.
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
//   P2.2 = Button (active-low)
//   P2.1 = LED (active-high): ON when frequency mismatch
//
// Notes:
// - No printf/sprintf, no float.
// - Small-angle approx for power:
//     sin(phi) ˜ phi, cos(phi) ˜ 1 - phi^2/2, phi in radians (Q15)
//   Works well when |phi| is not huge.
// - For 8051 safety: most multi-byte globals are xdata.

#include <EFM8LB1.h>

#define SYSCLK 72000000L
#define SARCLK 18000000L

// LCD
#define LCD_RS P1_7
#define LCD_E  P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0

// UI
#define MODE_BTN      P2_2
#define MISMATCH_LED  P2_1

// Signals
#define REF_SQ   P0_1
#define TEST_SQ  P0_2

// ADC constants
#define VDD_UV 3303500UL
#define ADC_FS 16383UL

// Fixed-point
#define RMS_NUM 7071UL
#define RMS_DEN 10000UL

// ===================== SMALL DATA (keep tiny) =====================
unsigned char overflow_count;
unsigned char mode = 0;
bit last_btn = 1;
bit mismatch = 0;

// ===================== XDATA: all large variables =====================
xdata unsigned long period_ref;
xdata unsigned long period_test;
xdata unsigned long dticks;
xdata unsigned long amp_ticks;

xdata unsigned int ref_amp_mV;
xdata unsigned int test_amp_mV;
xdata unsigned int ref_rms_mV;
xdata unsigned int test_rms_mV;

xdata long phase_cdeg;

xdata unsigned long gain_x10000;
xdata unsigned long ratio_q16_16;
xdata long db_centi;

// Power temporaries
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

	SFRPAGE=0x10;
	PFE0CN=0x20;
	SFRPAGE=0x00;

	CLKSEL=0x00; CLKSEL=0x00; while(!(CLKSEL&0x80));
	CLKSEL=0x03; CLKSEL=0x03; while(!(CLKSEL&0x80));

	XBR2=0x40;
	return 0;
}

// ===================== TIMER0 =====================
void TIMER0_Init(void)
{
	TMOD &= 0xF0;
	TMOD |= 0x01;
	TR0=0;
}

// ===================== TIMER3 =====================
void Timer3us(unsigned char us)
{
	unsigned char i;
	CKCON0|=0x40;
	TMR3RL=-(SYSCLK/1000000L);
	TMR3=TMR3RL;
	TMR3CN0=0x04;
	for(i=0;i<us;i++)
	{
		while(!(TMR3CN0&0x80));
		TMR3CN0&=~0x80;
	}
	TMR3CN0=0;
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
		if(s[i]==0) {
			MISMATCH_LED = 1;
			break;
		}
		LCD_data(s[i]);
	}
	for(;i<16;i++) LCD_data(' ');
}

// ===================== ADC =====================
void InitADC(void)
{
	ADEN=0;
	ADC0CN1=(0x2<<6);
	ADC0CF0=((SYSCLK/SARCLK)<<3);
	ADC0CF2=(0x1<<5)|(0x1F);
	ADEN=1;
}

void InitPinADC(unsigned char portno,unsigned char pinno)
{
	unsigned char mask;
	mask = 1<<pinno;
	SFRPAGE=0x20;
	if(portno==2){ P2MDIN&=(~mask); P2SKIP|=mask; }
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
	adc=ADC_read(pin);
	uv=(adc*VDD_UV)/ADC_FS;
	return (unsigned int)((uv+500)/1000);
}

// ===================== PERIOD =====================
unsigned long measure_period(bit pin)
{
	unsigned long ticks;

	TL0=0; TH0=0; TF0=0; overflow_count=0;
	while(pin!=0);
	while(pin!=1);
	TR0=1;
	while(pin!=0) if(TF0){TF0=0; overflow_count++;}
	while(pin!=1) if(TF0){TF0=0; overflow_count++;}
	TR0=0;

	ticks = ((unsigned long)overflow_count<<16)
	      | ((unsigned long)TH0<<8)
	      | TL0;

	return ticks;
}

// ===================== PHASE =====================
long phase_calc(unsigned long dt,unsigned long T)
{
	unsigned long t360;
	unsigned long deg;
	unsigned long rem;
	t360=dt*360UL;
	deg=t360/T;
	rem=t360%T;
	return (long)(deg*100UL+(rem*100UL)/T);
}

long wrap_phase(long p)
{
	while(p>18000L)p-=36000L;
	while(p<-18000L)p+=36000L;
	return p;
}

// ===================== LOG2 FOR dB (approx, lightweight) =====================
int log2_q8_8(unsigned long m)
{
	int e;
	unsigned long u,u2,u3;
	long ln;
	long frac;
	long total;

	e=0;
	if(m==0)return -32768;

	while(m>=(2UL<<16)){m>>=1;e++;}
	while(m<(1UL<<16)){m<<=1;e--;}

	u=m-(1UL<<16);
	u2=(u*u)>>16;
	u3=(u2*u)>>16;
	ln=(long)u-(long)(u2>>1)+(long)(u3/3UL);

	frac=(ln*94548L)>>16;
	total=((long)e<<16)+frac;
	return (int)(total>>8);
}

long db_centi_from_ratio_q16_16(unsigned long r)
{
	int l2;
	l2=log2_q8_8(r);
	return ((long)l2*60206L)/256L;
}

// ===================== DISPLAY HELPERS =====================
void clear_line(xdata char *s)
{
	unsigned char i;
	for(i=0;i<16;i++)s[i]=' ';
	s[16]=0;
}
void put_char(xdata char *s,unsigned char p,char c){if(p<16)s[p]=c;}
void put_str(xdata char *s,unsigned char p,const char *t){while(*t&&p<16)s[p++]=*t++;}

void put_mV(xdata char *s,unsigned char p,unsigned int mv)
{
	unsigned int v;
	unsigned int f;
	v=mv/1000;
	f=(mv%1000)/10;
	put_char(s,p,'0'+(v%10));
	put_char(s,p+1,'.');
	put_char(s,p+2,'0'+(f/10));
	put_char(s,p+3,'0'+(f%10));
}

// Signed centi-units as +ddd.dd (7 chars)
void put_centi_signed(xdata char *s, unsigned char p, long value_centi)
{
	char sign;
	unsigned long a;
	unsigned int d;
	unsigned int f;

	sign = '+';
	if(value_centi < 0)
	{
		sign = '-';
		a = (unsigned long)(-value_centi);
	}
	else
	{
		a = (unsigned long)value_centi;
	}

	d = (unsigned int)(a / 100UL);
	f = (unsigned int)(a % 100UL);

	put_char(s, p+0, sign);
	put_char(s, p+1, '0' + ((d/100)%10));
	put_char(s, p+2, '0' + ((d/10)%10));
	put_char(s, p+3, '0' + (d%10));
	put_char(s, p+4, '.');
	put_char(s, p+5, '0' + (f/10));
	put_char(s, p+6, '0' + (f%10));
}

void put_gain(xdata char *s,unsigned char p,unsigned long g)
{
	unsigned int i;
	unsigned int f;
	i=(unsigned int)(g/10000UL);
	f=(unsigned int)(g%10000UL);
	put_char(s,p,'0'+(i%10));
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
	if(cdb < 0)
	{
		sign = '-';
		a = (unsigned long)(-cdb);
	}
	else
	{
		a = (unsigned long)cdb;
	}

	d = (unsigned int)(a / 100UL);
	f = (unsigned int)(a % 100UL);

	put_char(s, p+0, sign);
	put_char(s, p+1, '0' + ((d/10)%10));
	put_char(s, p+2, '0' + (d%10));
	put_char(s, p+3, '.');
	put_char(s, p+4, '0' + (f/10));
	put_char(s, p+5, '0' + (f%10));
}

// ===================== POWER HELPERS =====================
// Convert centi-deg to radians in Q15 using constant: (pi/180)*32768 ˜ 572
long phase_to_rad_q15(long cdeg)
{
	return (cdeg * 572L) / 100L;
}

// ===================== MAIN =====================
void main(void)
{
	unsigned char i;

	TIMER0_Init();

	SFRPAGE=0x20;
	P2MDOUT|=0x02;     // P2.1 push-pull
	SFRPAGE=0x00;
	MISMATCH_LED = 0;

	InitPinADC(2,5);
	InitPinADC(2,6);
	InitADC();
	LCD_init();

	while(1)
	{
		// Button edge-detect
		if((MODE_BTN==0)&&(last_btn==1))mode=(mode+1)&0x03;
		last_btn=MODE_BTN;

		// Periods
		period_ref=measure_period(REF_SQ);
		period_test=measure_period(TEST_SQ);

		// Mismatch LED hysteresis
		{
			long diff;
			unsigned long ad;
			diff=(long)period_test-(long)period_ref;
			ad=(diff<0)?(unsigned long)(-diff):(unsigned long)diff;

			if(!mismatch){if(ad*400UL>period_ref)mismatch=1;}
			else{if(ad*770UL<period_ref)mismatch=0;}
			MISMATCH_LED=mismatch;
		}

		amp_ticks=period_ref/4;

		// REF RMS
		while(REF_SQ!=0);while(REF_SQ!=1);
		TL0=0;TH0=0;TF0=0;overflow_count=0;TR0=1;
		while((((unsigned long)overflow_count<<16)|((unsigned long)TH0<<8)|TL0)<amp_ticks)
			if(TF0){TF0=0;overflow_count++;}
		TR0=0;
		ref_amp_mV=mV_read(QFP32_MUX_P2_6);
		ref_rms_mV=(unsigned int)(((unsigned long)ref_amp_mV*RMS_NUM)/RMS_DEN);

		// TEST RMS
		while(TEST_SQ!=0);while(TEST_SQ!=1);
		TL0=0;TH0=0;TF0=0;overflow_count=0;TR0=1;
		while((((unsigned long)overflow_count<<16)|((unsigned long)TH0<<8)|TL0)<amp_ticks)
			if(TF0){TF0=0;overflow_count++;}
		TR0=0;
		test_amp_mV=mV_read(QFP32_MUX_P2_5);
		test_rms_mV=(unsigned int)(((unsigned long)test_amp_mV*RMS_NUM)/RMS_DEN);

		// Phase (reference edge then test leads/lags)
		while(REF_SQ!=0);while(REF_SQ!=1);
		{
			bit st;
			st=TEST_SQ;

			TL0=0;TH0=0;TF0=0;overflow_count=0;TR0=1;
			if(st==0)
			{
				while(TEST_SQ!=1)if(TF0){TF0=0;overflow_count++;}
				TR0=0;
				dticks=((unsigned long)overflow_count<<16)|((unsigned long)TH0<<8)|TL0;
				phase_cdeg=-phase_calc(dticks,period_ref);
			}
			else
			{
				while(TEST_SQ!=0)if(TF0){TF0=0;overflow_count++;}
				while(TEST_SQ!=1)if(TF0){TF0=0;overflow_count++;}
				TR0=0;
				dticks=((unsigned long)overflow_count<<16)|((unsigned long)TH0<<8)|TL0;
				dticks=period_ref-dticks;
				phase_cdeg=phase_calc(dticks,period_ref);
			}
		}
		phase_cdeg=wrap_phase(phase_cdeg);

		// Display
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
			if(ref_rms_mV>0)
			{
				gain_x10000=((unsigned long)test_rms_mV*10000UL)/ref_rms_mV;
				ratio_q16_16=((unsigned long)test_rms_mV<<16)/ref_rms_mV;
				db_centi=db_centi_from_ratio_q16_16(ratio_q16_16);
			}
			else{gain_x10000=0;db_centi=0;}

			put_str(line1,0,"G=");
			put_gain(line1,2,gain_x10000);

			put_str(line2,0,"dB=");
			put_db(line2,3,db_centi);
		}
		else if(mode==2)
		{
			// Power (P and Q)
			VI_mW = ((long)ref_rms_mV * (long)test_rms_mV) / 1000L;

			phi_rad_q15 = phase_to_rad_q15(phase_cdeg);

			// small-angle sin/cos in Q15
			sinphi_q15 = phi_rad_q15;
			phi2_q15 = (phi_rad_q15 * phi_rad_q15) >> 15;
			cosphi_q15 = 32768L - (phi2_q15 >> 1);

			real_power_mW = (VI_mW * cosphi_q15) >> 15;
			reactive_power_mW = (VI_mW * sinphi_q15) >> 15;

			// display as centi-W: 0.01 W = 10 mW => centiW = mW/10
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

		// gentle update rate
		for(i=0;i<10;i++) waitms(10);
	}
}