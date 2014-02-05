/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN			-100
#define BUFFER_DUMP			4096

#define FREQUENCIES_LIMIT		1000


#define Fe 48000.0
#define Freqh 4800.0/Fe*2.0*M_PI
#define Freql 2400.0/Fe*2.0*M_PI
#define BITLEN ((int)Fe/1200)

/* ACARS defines */
#define VFOPLL 0.7e-3
#define BITPLL 0.2
#define SYN 0x16
#define SOH 0x01
#define POLY 0x1021

static struct bstat_s {
	float hsample[BITLEN];
	float lsample[BITLEN];
	float isample[BITLEN];
	float qsample[BITLEN];
	float csample[BITLEN];
	int is;
	int clock;
	float lin;
	float phih,phil;
	float dfh,dfl;
	float pC,ppC;
	int sgI, sgQ;
	float ea;
} bstat[2];

struct mstat_s {
	enum { HEADL, HEADF, BSYNC1, BSYNC2, SYN1, SYN2, SOH1, TXT, CRC1,
		    CRC2, END } state;
	int ind;
	unsigned short crc;
	char txt[256];
	unsigned char c1,c2,c3;
} mstat[2];




struct acars_flight {
	char *flightid;
	char *from;
	char *to;
	char *airline;
};
struct acars_flight acars_flights[160000];


struct acars_aircraft {
	char *registration;
	char *modes;
	char *manufacturer;
	char *model;
};
struct acars_aircraft acars_aircrafts[500000];


struct acars_airport {
	char *name;
	char *city;
	char *country;
	char *code;
};
struct acars_airport acars_airports[10000];



struct acars_ml {
	char *ml_code;
	char *ml_label;
};

struct acars_ml acars_mls[16000];

/*
struct acars_airlines {
	char *al_code;
	char *al_label;
};

struct acars_airlines acars_airliness[16000];
*/



static pthread_t demod_thread;
static pthread_cond_t data_ready;   /* shared buffer filled */
static pthread_rwlock_t data_rw;    /* lock for shared buffer */
static pthread_mutex_t data_mutex;  /* because conds are dumb */

static pthread_mutex_t dataset_mutex;

static volatile int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH;
static float h[BITLEN];


static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

static int debug_hop=0;
static int current_freq = 0;

struct fm_state
{
	int      now_r, now_j;
	int      pre_r, pre_j;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int      exit_flag;
	uint8_t  buf[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int      signal[MAXIMUM_BUF_LENGTH];  /* 16 bit signed i/q pairs */
	int16_t  signal2[MAXIMUM_BUF_LENGTH]; /* signal has lowpass, signal2 has demod */
	int      signal_len;
	int      signal2_len;
	FILE     *file;
	int      edge;
	uint32_t freqs[FREQUENCIES_LIMIT];
	int      freq_len;
	int      freq_now;
	uint32_t sample_rate;
	int      output_rate;
	int      fir_enable;
	int      fir[256];  /* fir_len == downsample */
	int      fir_sum;
	int      custom_atan;
	int      deemph, deemph_a;
	int      now_lpr;
	int      prev_lpr_index;
	int      dc_block, dc_avg;
	void     (*mode_demod)(struct fm_state*);
};

typedef struct {
	unsigned char mode;
	unsigned char addr[8];
	unsigned char ack;
	unsigned char label[3];
	unsigned char bid;
	unsigned char no[5];
	unsigned char fid[7];
	char txt[256];
	int crc;
} msg_t;

// ACARS decoder variables
static long rx_idx;
int c;
msg_t msgl;
unsigned char rl;
int nbitl = 0;
int nrbitl = 8;


void usage(void)
{
	fprintf(stderr,
		"rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
		"Use:\tnew_rtl_acars -f freq [-options] \n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t (use multiple -f for scanning, requires squelch)\n"
		"\t (ranges supported, -f 118M:137M:25k)\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t[-o oversampling (default: 1, 4 recommended)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-r squelch debug mode ]\n"
		"\t[-t squelch_delay (default: 0)]\n"
		"\t (+values will mute/scan, -values will exit)\n"
		"\t[-F enables Hamming FIR (default: off/square)]\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		//rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	//rtlsdr_cancel_async(dev);
}
#endif


void load_flights(void)
{
	FILE *f = fopen("datasets/flightroute2.txt", "r");
	if (!f) {
		fprintf(stderr, "Warning: datasets/flightroute2.txt data source not found\n");
		acars_flights[0].flightid = NULL;
		return;
	}

	int i = 0;

	char *line = NULL;
	size_t len = 0;
	while (getline(&line, &len, f) != -1) {
		char *item = line;
		char *tabpos;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_flights[i].flightid = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_flights[i].from = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_flights[i].to = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		tabpos[0] = 0;
		acars_flights[i].airline = strdup(item);

		i++;
	}

	acars_flights[i].flightid = NULL;
	fclose(f);
	printf("Loaded: %i flights from dataset.....\n", i);
}




void load_airports(void)
{
	FILE *f = fopen("datasets/airports.txt", "r");
	if (!f) {
		fprintf(stderr, "Warning: datasets/airports.txt data source not found\n");
		acars_airports[0].code = NULL;
		return;
	}

	int i = 0;

	char *line = NULL;
	size_t len = 0;
	while (getline(&line, &len, f) != -1) {
		char *item = line;
		char *tabpos;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_airports[i].name = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_airports[i].city = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_airports[i].country = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		tabpos[0] = 0;
		acars_airports[i].code = strdup(item);

		i++;
	}

	acars_airports[i].code = NULL;
	fclose(f);
	printf("Loaded: %i airports from dataset.....\n", i);
}


void load_aircrafts(void)
{
	FILE *f = fopen("datasets/aircrafts.txt", "r");
	if (!f) {
		fprintf(stderr, "Warning: datasets/aircrafts.txt data source not found\n");
		acars_aircrafts[0].registration = NULL;
		return;
	}

	int i = 0;

	char *line = NULL;
	size_t len = 0;
	while (getline(&line, &len, f) != -1) {
		char *item = line;
		char *tabpos;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_aircrafts[i].registration = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_aircrafts[i].modes = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_aircrafts[i].manufacturer = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		if (!tabpos) {tabpos = malloc(1);tabpos[0] = 0;}
		acars_aircrafts[i].model = strdup(item);
		i++;
	}

	acars_aircrafts[i].registration = NULL;
	fclose(f);
	printf("Loaded: %i aircrafts from dataset.....\n", i);
}





void load_message_labels(void)
{
	FILE *f = fopen("datasets/acars_mls.txt", "r");
	if (!f) {
		fprintf(stderr, "Warning: datasets/acars_mls.txt data source not found\n");
		acars_mls[0].ml_code = NULL;
		return;
	}

	int i = 0;

	char *line = NULL;
	size_t len = 0;
	while (getline(&line, &len, f) != -1) {
		char *item = line;
		char *tabpos;

		tabpos = strchr(item, '\t');
		if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
		tabpos[0] = 0;
		acars_mls[i].ml_code = strdup(item);
		item = tabpos + 1;

		tabpos = strchr(item, '\t');
		tabpos[0] = 0;
		acars_mls[i].ml_label = strdup(item);

		i++;
	}

	acars_mls[i].ml_code = NULL;
	fclose(f);
	printf("Loaded: %i ACARS message labels from dataset.....\n", i);
}




int getbit(short sample, unsigned char *outbits, int ch) {
	int i, bt;
	float in, in2;
	float C;
	float I, Q;
	float oscl, osch;
	struct bstat_s *st;

	bt = 0;
	st = &bstat[ch];

	in = (float) sample;
	st->lin = 0.003 * fabs(in) + 0.997 * st->lin;
	in /= st->lin;
	in2 = in * in;

	st->is--;
	if (st->is < 0)
		st->is = BITLEN - 1;

	/* VFOs */
	st->phih += Freqh - VFOPLL * st->dfh;
	if (st->phih >= 4.0 * M_PI)
		st->phih -= 4.0 * M_PI;
	st->hsample[st->is] = in2 * sin(st->phih);
	for (i = 0, st->dfh = 0.0; i < BITLEN / 2; i++) {
		st->dfh += st->hsample[(st->is + i) % BITLEN];
	}
	osch = cos(st->phih / 2.0);

	st->phil += Freql - VFOPLL * st->dfl;
	if (st->phil >= 4.0 * M_PI)
		st->phil -= 4.0 * M_PI;
	st->lsample[st->is] = in2 * sin(st->phil);
	for (i = 0, st->dfl = 0.0; i < BITLEN / 2; i++) {
		st->dfl += st->lsample[(st->is + i) % BITLEN];
	}
	oscl = cos(st->phil / 2.0);

	/* mix */
	st->isample[st->is] = in * (oscl + osch);
	st->qsample[st->is] = in * (oscl - osch);
	st->csample[st->is] = oscl * osch;


	/* bit clock */
	st->clock++;
	if (st->clock >= BITLEN/4 + st->ea) {
		st->clock = 0;

		/*  clock filter  */
		for (i = 0, C = 0.0; i < BITLEN; i++) {
			C += h[i] * st->csample[(st->is + i) % BITLEN];
		}

		if (st->pC < C && st->pC < st->ppC) {

			/* integrator */
			for (i = 0, Q = 0.0; i < BITLEN; i++) {
				Q += st->qsample[(st->is + i) % BITLEN];
			}

			if (st->sgQ == 0) {
				if (Q < 0)
					st->sgQ = -1;
				else
					st->sgQ = 1;
			}

			*outbits =
			    ((*outbits) >> 1) | (unsigned
						 char) ((Q * st->sgQ >
							 0) ? 0x80 : 0);
			bt = 1;

			st->ea = -BITPLL * (C - st->ppC);
			if(st->ea > 2.0) st->ea=2.0;
			if(st->ea < -2.0) st->ea=-2.0;
		}
		if (st->pC > C && st->pC > st->ppC) {

			/* integrator */
			for (i = 0, I = 0.0; i < BITLEN; i++) {
				I += st->isample[(st->is + i) % BITLEN];
			}

			if (st->sgI == 0) {
				if (I < 0)
					st->sgI = -1;
				else
					st->sgI = 1;
			}

			*outbits =
			    ((*outbits) >> 1) | (unsigned
						 char) ((I * st->sgI >
							 0) ? 0x80 : 0);
			bt = 1;

			st->ea = BITPLL * (C - st->ppC);
			if(st->ea > 2.0) st->ea=2.0;
			if(st->ea < -2.0) st->ea=-2.0;
		}
		st->ppC = st->pC;
		st->pC = C;
	}
	return bt;
}

void init_bits(void) {
	int i;
	for (i = 0; i < BITLEN; i++)
		h[i] = sin(2.0 * M_PI * (float) i / (float) BITLEN);

	for (i = 0; i < BITLEN; i++) {
		bstat[0].hsample[i] = bstat[0].lsample[i] =
		    bstat[0].isample[i] = bstat[0].qsample[i] =
		    bstat[0].csample[i] = 0.0;
		bstat[1].hsample[i] = bstat[1].lsample[i] =
		    bstat[1].isample[i] = bstat[1].qsample[i] =
		    bstat[1].csample[i] = 0.0;
	}
	bstat[0].is = bstat[0].clock = bstat[0].sgI = bstat[0].sgQ = 0;
	bstat[1].is = bstat[1].clock = bstat[1].sgI = bstat[1].sgQ = 0;
	bstat[0].phih = bstat[0].phil = bstat[0].dfh = bstat[0].dfl =
	    bstat[0].pC = bstat[0].ppC = bstat[0].ea = 0.0;
	bstat[1].phih = bstat[1].phil = bstat[1].dfh = bstat[1].dfl =
	    bstat[1].pC = bstat[1].ppC = bstat[1].ea = 0.0;
	bstat[0].lin=bstat[1].lin=1.0;

}

void init_mesg(void) {
	mstat[0].state = mstat[1].state = HEADL;
}

void resetbits(int ch) {
	bstat[ch].sgI = bstat[ch].sgQ = 0;
}

ssize_t getline(char **linep, size_t *np, FILE *stream) {
  char *p = NULL;
  size_t i = 0;

  if (!linep || !np) {
    errno = EINVAL;
    return -1;
  }

  if (!(*linep) || !(*np)) {
    *np = 120;
    *linep = (char *)malloc(*np);
    if (!(*linep)) {
      return -1;
    }
  }

  flockfile(stream);

  p = *linep;
  int ch;
  for (ch = 0; (ch = getc_unlocked(stream)) != EOF;) {
    if (i > *np) {
      /* Grow *linep. */
      size_t m = *np * 2;
      char *s = (char *)realloc(*linep, m);

      if (!s) {
        int error = errno;
        funlockfile(stream);
        errno = error;
        return -1;
      }

      *linep = s;
      *np = m;
    }

    p[i] = ch;
    if ('\n' == ch) break;
    i += 1;
  }
  funlockfile(stream);

  /* Null-terminate the string. */
  if (i > *np) {
    /* Grow *linep. */
      size_t m = *np * 2;
      char *s = (char *)realloc(*linep, m);

      if (!s) {
        return -1;
      }

      *linep = s;
      *np = m;
  }

  p[i + 1] = '\0';
  return ((i > 0)? (ssize_t) i : -1);
}

static void update_crc(unsigned short *crc, unsigned char ch) {
	unsigned char v;
	unsigned int i;
	unsigned short flag;

	v = 1;
	for (i = 0; i < 8; i++) {
		flag = (*crc & 0x8000);
		*crc = *crc << 1;

		if (ch & v)
			*crc = *crc + 1;

		if (flag != 0)
			*crc = *crc ^ POLY;

		v = v << 1;
	}
}

static int build_mesg(char *txt, int len, msg_t * msg) {
	int i, k;
	char r;

	/* remove special chars */
	for (i = 0; i < len; i++) {
		r = txt[i];
		if (r < ' ' && r != 0x0d && r != 0x0a)
			r = '.'; // was 0xa4 AR CHANGE: Set other placeholder
		txt[i] = r;
	}
	txt[i] = '\0';

	/* fill msg struct */
	k = 0;
	msg->mode = txt[k];
	k++;

	for (i = 0; i < 7; i++, k++) {
		msg->addr[i] = txt[k];
	}
	msg->addr[7] = '\0';

	/* ACK/NAK */
	msg->ack = txt[k];
	k++;

	msg->label[0] = txt[k];
	k++;
	msg->label[1] = txt[k];
	k++;
	msg->label[2] = '\0';

	msg->bid = txt[k];
	k++;
	k++;

	for (i = 0; i < 4; i++, k++) {
		msg->no[i] = txt[k];
	}
	msg->no[4] = '\0';

	for (i = 0; i < 6; i++, k++) {
		msg->fid[i] = txt[k];
	}
	msg->fid[6] = '\0';

	strncpy(msg->txt, &(txt[k]),255);
	msg->txt[255]='\0';

	return 1;
}


int bitsdiff(unsigned char src, unsigned char dst)
{
    unsigned char diff = 0;
    int bits = 0;

    diff = src^dst;
    if (diff==0) return 0;
    /* the slow, boring way to do that */
    while (diff!=0) {
	if (diff&1) bits++;
	diff>>=1;
    }
    return bits;
}


int getmesg(unsigned char r, msg_t * msg, int ch) {
	unsigned short mcrc;
	unsigned char mtxt[256];
	struct mstat_s *st;
	st = &(mstat[ch]);

	do {
		switch (st->state) {
		case HEADL:
			if (r == 0xff) {
				st->state = HEADF;
				return 8;
			}
			resetbits(ch);
			return 8;
			break;
		case HEADF:
			if (r != 0xff) {
				int i;
				unsigned char m;

				for (i = 0, m = 1; i < 7; i++, m = m << 1) {
					if (!(r & m))
						break;
				}
				if (i < 2) {
					st->state = HEADL;
					break;
				}
				st->state = BSYNC1;
				st->ind = 0;
				if (i != 2)
					return (i - 2);
				break;
			}
			return 6;
		case BSYNC1:
			if (r != 0x80 + '+')
				st->ind++;
			st->state = BSYNC2;
			return 8;
		case BSYNC2:
			if (r != '*')
				st->ind++;
			st->state = SYN1;
			return 8;
		case SYN1:
			if (r != SYN)
				st->ind++;
			st->state = SYN2;
			return 8;
		case SYN2:
			if (r != SYN)
				st->ind++;
			st->state = SOH1;
			return 8;
		case SOH1:
			if (r != SOH)
				st->ind++;
			if (st->ind > 2) {
				st->state = HEADL;
				break;
			}
			st->state = TXT;
			st->ind = 0;
			st->crc = 0;
			memset(st->txt,0,256);
			return 8;
		case TXT:
			update_crc(&st->crc, r);
			st->c1 = r;
			r = r & 0x7f;
			if (r == 0x03 || r == 0x17) {
				st->state = CRC1;
				return 8;
			}
			st->txt[st->ind] = r;
			st->ind++;
			if (st->ind > 243) {
				st->state = HEADL;
				break;
			}
			return 8;
		case CRC1:
			update_crc(&st->crc, r);
			st->c2 = r;
			st->state = CRC2;
			return 8;
		case CRC2:
			update_crc(&st->crc, r);
			st->c3 = r;
			st->state = END;
			return 8;
		case END:
			st->state = HEADL;
			if (st->crc == 0) {
				msg->crc = 0;
				build_mesg(st->txt, st->ind, msg);
				return 0;
			}
			else {
				int c1,c2,c3,c4,c5,c6,err;
				msg->crc = 1;
				// Correct CRC, single bit error
				for (c1=0;c1<st->ind;c1++)
				for (c2=0;c2<8;c2++)
				{
				    mcrc = 0;
				    memset(mtxt,0,256);
				    for (c3=0;c3<st->ind;c3++)
				    {
				        if (c1==c3) {update_crc(&mcrc,st->txt[c3]^((unsigned char)1<<(unsigned char)c2));mtxt[c3]=(st->txt[c3]^((unsigned char)1<<(unsigned char)c2));}
				        else {update_crc(&mcrc,st->txt[c3]);mtxt[c3]=st->txt[c3];}
				    }
				    update_crc(&mcrc, st->c1);
				    update_crc(&mcrc, st->c2);
				    update_crc(&mcrc, st->c3);
				    if (mcrc==0) 
				    {
					memcpy(st->txt,mtxt,st->ind);
					build_mesg(st->txt, st->ind, msg);
					// Do some heuristic checks
					int a;
					err = 0;
					for (a=0;a<7;a++) if ((msg->addr[a]<32)||(msg->addr[a]>127)) err = 1;
					if (strlen(msg->fid)>1)
					for (a=0;a<6;a++) if ((msg->fid[a]<32)||(msg->fid[a]>90)) err = 1;
					if (!err) return 0;
				    }
				}
				// Correct CRC, two bit error
				memset(mtxt,0,256);
				for (c1=0;c1<st->ind;c1++)
				for (c2=0;c2<8;c2++)
				for (c4=0;c4<st->ind;c4++)
				for (c5=0;c5<8;c5++)
				{
				    mcrc = 0;
				    for (c3=0;c3<st->ind;c3++)
				    {
				        if (c1==c3) {update_crc(&mcrc,st->txt[c3]^((unsigned char)1<<(unsigned char)c2));mtxt[c3]=(st->txt[c3]^((unsigned char)1<<(unsigned char)c2));}
				        else if (c4==c3) {update_crc(&mcrc,st->txt[c3]^((unsigned char)1<<(unsigned char)c5));mtxt[c3]=(st->txt[c3]^((unsigned char)1<<(unsigned char)c5));}
				        else {update_crc(&mcrc,st->txt[c3]);mtxt[c3]=st->txt[c3];}
				    }
				    update_crc(&mcrc, st->c1);
				    update_crc(&mcrc, st->c2);
				    update_crc(&mcrc, st->c3);
				    if (mcrc==0) 
				    {
					memcpy(st->txt,mtxt,st->ind);
					build_mesg(st->txt, st->ind, msg);
					// Do some heuristic checks
					int a;
					err = 0;
					for (a=0;a<7;a++) if ((msg->addr[a]<32)||(msg->addr[a]>127)) err = 1;
					if (strlen(msg->fid)>1)
					for (a=0;a<6;a++) if ((msg->fid[a]<32)||(msg->fid[a]>90)) err = 1;
					if (!err) return 0;
				    }
				}
				st->state = HEADL;return 8;
			}
			st->state = HEADL;return 8;
		}
	} while (1);
}



void process_qv(char *txt)
{
    switch (txt[0])
    {
	case '1': printf("\nAutotune reject reason: Contrary to airline preference\n");break;
	case '2': printf("\nATN session in progress\n");break;
	case '3': printf("\nAutotune uplink format error\n");break;
	default: printf("\nAutotune reject reason: unknown\n");
    }
}



void process_sa(char *txt)
{
    printf("Version: %c\n",txt[0]);
    if (txt[1]=='E') printf("Link state: Established\n");
    else if (txt[1]=='L') printf("Link state: Lost\n");
    else printf("Link state: Unknown\n");
    switch (txt[2])
    {
	case 'V': printf("Link type: VHF ACARS\n");break;
	case 'S': printf("Link type: Generic SATCOM\n");break;
	case 'H': printf("Link type: HF\n");break;
	case 'G': printf("Link type: GlobalStar SATCOM\n");break;
	case 'C': printf("Link type: ICO SATCOM\n");break;
	case '2': printf("Link type: VDL Mode 2\n");break;
	case 'X': printf("Link type: Inmarsat Aero\n");break;
	case 'I': printf("Link type: Irridium SATCOM\n");break;
	default:  printf("Link type: Unknown\n");
    }
    printf("Event occured at: %c%c:%c%c:%c%c\n",txt[3],txt[4],txt[5],txt[6],txt[7],txt[8]);
}



void process_5u(char *txt)
{
    char airport[4];
    int cur=0;
    int cur2=0;
    int i=0;
    
    printf("Weather report requested from: ");
    while (txt[cur]!=0)
    {
	if ((txt[cur]<='Z')&&(txt[cur]>='A'))
	{
	    airport[cur2]=txt[cur];
	    cur2++;
	    if (cur2==4)
	    {
		i=0;
		while(acars_airports[i].code)
		{
		    const char *regtmp = (const char *) airport;
		    while (regtmp[0] == '.')
			regtmp++;
		    if(!strcmp(acars_airports[i].code, regtmp))
		    {
			printf("%s (%s) ",acars_airports[i].name,acars_airports[i].city);
		    }
		    i++;
		}
		cur2 = 0;
	    }
	}
	else cur2=0;
	cur++;
    }
    printf("\n");
}

void process_q1(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("OFF event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("ON event occured at: %c%c:%c%c\n",txt[12],txt[13],txt[14],txt[15]);
    printf("IN event occured at: %c%c:%c%c\n",txt[16],txt[17],txt[18],txt[19]);
    printf("Fuel: %c%c%c%c\n",txt[20],txt[21],txt[22],txt[23]);
    printf("Destination station: %c%c%c%c\n",txt[24],txt[25],txt[26],txt[27]);
}

void process_q2(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("ETA: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Fuel: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qa(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
    printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
}

void process_qb(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OFF event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}

void process_qc(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("ON event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}

void process_qd(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("IN event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
    printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
    printf("Captain/First officer ID: %c\n",txt[17]);
}


void process_qe(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
    printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
    printf("Destination station: %c%c%c\n",txt[17],txt[18],txt[19]);
}

void process_qf(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OFF event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Destination station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}


void process_qg(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Return IN event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qh(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}


void process_qk(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("ON event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Destination station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_ql(char *txt)
{
    printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("IN event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Fuel quantity: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("Captain/First officer ID: %c\n",txt[12]);
    printf("Departure station: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
}

void process_qm(char *txt)
{
    printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("Fuel quantity: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("Departure station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("Category of landing: %c\n",txt[12]);
}

void process_qn(char *txt)
{
    printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("New destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("ETA at diversion station: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("Fuel quantity: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
    printf("Flight segment originating station: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
}


void process_qp(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[7],txt[8],txt[10],txt[11]);
    printf("Boarded fuel: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
}

void process_qq(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("OFF event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qr(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("ON event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qs(char *txt)
{
    printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("New destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("IN event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("Fuel quantity: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
    printf("Flight segment originating station: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
    printf("Captain/First officer ID: %c\n",txt[20]);
}

void process_qt(char *txt)
{
    printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
    printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
    printf("OUT event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
    printf("Return IN event occured at: %c%c:%c%c\n",txt[12],txt[13],txt[14],txt[15]);
    printf("Fuel onboard: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
}

void process_57(char *txt)
{
    printf("Current position: %c%c%c%c%c\n",txt[0],txt[1],txt[2],txt[3],txt[4]);
    printf("Current time: %c%c:%c%c\n",txt[5],txt[6],txt[7],txt[8]);
    printf("Flight level: %c%c%c\n",txt[9],txt[10],txt[11]);
    printf("Next report point: %c%c%c%c%c\n",txt[12],txt[13],txt[14],txt[15],txt[16]);
    printf("Time over: %c%c:%c%c\n",txt[17],txt[18],txt[19],txt[20]);
    printf("Fuel onboard: %c%c%c%c\n",txt[21],txt[22],txt[23],txt[24]);
    printf("Static air temp: %c%c%c\n",txt[25],txt[26],txt[27]);
    printf("Wind direction: %c%c%c deg.\n",txt[28],txt[29],txt[30]);
    printf("Wind speed: %c%c%c knots.\n",txt[31],txt[32],txt[33]);
    printf("Sky condition: %c%c%c%c%c%c%c%c\n",txt[34],txt[35],txt[36],txt[37],txt[38],txt[39],txt[40],txt[41]);
    printf("Turbulence: %c%c%c%c%c%c%c%c\n",txt[42],txt[43],txt[44],txt[45],txt[46],txt[47],txt[48],txt[49]);
    printf("Cruising speed: %c%c%c%c%c\n",txt[50],txt[51],txt[52],txt[54],txt[54]);
}


void process_h1(char *txt)
{
    if (!strncmp(txt,"#DF",3)) printf("Source: Digital Flight Data Acquisition Unit\n");
    if (!strncmp(txt,"#CF",3)) printf("Source: Central Fault Display\n");
    if (!strncmp(txt,"#M1",3)) printf("Source: Flight Management Computer, Left\n");
    if (!strncmp(txt,"#M2",3)) printf("Source: Flight Management Computer, Right\n");
    if (!strncmp(txt,"#M3",3)) printf("Source: Flight Management Computer, Center\n");
    if (!strncmp(txt,"#MD",3)) printf("Source: Flight Management Computer, Selected\n");
    if (!strncmp(txt,"#EC",3)) printf("Source: Engine Display System\n");
    if (!strncmp(txt,"#EI",3)) printf("Source: Engine Indicating System\n");
    if (!strncmp(txt,"#PS",3)) printf("Source: Keyboard/Display Unit\n");
    if (!strncmp(txt,"#S1",3)) printf("Source: SDU, Left\n");
    if (!strncmp(txt,"#S2",3)) printf("Source: SDU, Right\n");
    if (!strncmp(txt,"#SD",3)) printf("Source: SDU, Selected\n");
    if (!strncmp(txt,"#T",2)) printf("Source: Cabin Terminal Message\n");
    if (!strncmp(txt,"#WO",3)) printf("Source: Weather Observation Report\n");
}


void process_54(char *txt)
{
    printf("Frequency (MHZ): %c%c%c.%c%c%c\n",txt[0],txt[1],txt[2],txt[3],txt[4],txt[5]);
}






int is_flight_num(const char *text)
{
    int a=0;
    int ok=1,dig=0;

    while (a<6) 
    {
	if (!((text[a]=='-') || (text[a]=='.') || ((text[a]<='Z') && (text[a]>='A')) || ((text[a]>='0') && (text[a]<='9')))) ok = 0;
	a++;
    }
    if (!((text[2]>='0') && (text[a]<='9'))) ok = 0;
    for (a=3;a<6;a++) if (((text[2]>='0') && (text[a]<='9'))) dig = 1;
    if (dig==0) ok=0;
    return ok;
}




void print_mesg(msg_t * msg) {
	time_t t;
	struct tm *tmp;
	char pos[128];

	long i=0;

	printf("\n[BEGIN_MESSAGE]----------------------------------------------------------\n\n");
	printf("RX_IDX: %ld\n", rx_idx);
	if (msg->crc) printf("CRC: Bad, corrected\n");
	else printf("CRC: Correct\n");
	t = time(NULL);
	tmp = localtime(&t);
	printf("Timestamp: %02d/%02d/%04d %02d:%02d\n",	     tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year + 1900,
	     tmp->tm_hour, tmp->tm_min);
	printf("ACARS mode: %c \n", msg->mode);
	printf("Message label: %s ", msg->label);

	i=0;
	while(acars_mls[i].ml_code){
		if(!strcmp(acars_mls[i].ml_code, (const char*)msg->label)){
			printf("(%s)\n",acars_mls[i].ml_label);
		}
		i++;
	}

	printf("Aircraft reg: %s, ", msg->addr);
	printf("flight id: %s\n", msg->fid);
	i=0;
	if ((msg->addr)&&(strlen(msg->addr)<8)&&(strlen(msg->addr)>1)&&(strcmp(msg->addr,".......")!=0))
	{
	    while(acars_aircrafts[i].registration)
	    {
		char regtmp[8];
		memset(regtmp,0,8);
		int ind = 0;
		while ((ind<8)&&(msg->addr[ind]=='.')) ind++;
		strcpy(regtmp,&msg->addr[ind]);

		int len = strlen(regtmp);
		if ((len>0)&&(!strncmp(acars_aircrafts[i].registration, regtmp,len))){
			printf("Aircraft: %s \n",acars_aircrafts[i].manufacturer);
			printf("Registration: %s \n",acars_aircrafts[i].registration);
			printf("Mode-S ID: %s\n",acars_aircrafts[i].modes);
			goto aircraft_finished;
		}
		i++;
	    }
	}

aircraft_finished:

	i=0;
	int found=0;
	int found2=0;
	int found3=0;
	
	char regtmp[8];
	memset(regtmp,0,8);
	int ind = 0;
	regtmp[0]=msg->fid[0];
	regtmp[1]=msg->fid[1];
	ind = 1;
	int correct = is_flight_num(msg->fid);
	while ((ind<7)/*&&(msg->fid[ind]=='0')*/) 
	{
	    regtmp[2]='0';
	    if (ind>1) strncpy(&regtmp[3], &msg->fid[ind], 7-ind);
	    else strncpy(&regtmp[2], &msg->fid[ind+1], 4);
	    ind++;
	    if (strlen(msg->fid)>1) while(acars_flights[i].flightid){
		if ((!found)&&(!strncmp(acars_flights[i].flightid, regtmp,2))&&(correct)) {
		    printf("Airline: %s \n",acars_flights[i].airline);
		    found++;
		}

		if ((correct)&&(!strncmp(acars_flights[i].flightid, regtmp,2)) && (!strncmp(acars_flights[i].flightid+3, regtmp+3,strlen(regtmp+3)))&&(strlen(regtmp+3)>0))
		{
			long x = 0;
			while((acars_airports[x].code)&&(found2==0)){
				if(!strcmp(acars_airports[x].code, acars_flights[i].from)){
					printf("From: %s - %s (%s, %s) \n",acars_airports[x].code,acars_airports[x].name,acars_airports[x].city,acars_airports[x].country);
					found2++;
					break;
				}
				x++;
			}
			x=0;
			while((acars_airports[x].code)&&(found3==0)){
				if(!strcmp(acars_airports[x].code, acars_flights[i].to)){
					printf("To: %s - %s (%s, %s) \n",acars_airports[x].code,acars_airports[x].name,acars_airports[x].city,acars_airports[x].country);
					found3++;
					break;
				}
				x++;
			}
		}
		i++;
	    }
	}


	printf("\nBlock id: %d, ", (int) msg->bid);
	printf(" msg. no: %s\n", msg->no);
	if (!strcmp(msg->label,"QV")) process_qv(msg->txt);
	if (!strcmp(msg->label,"5U")) process_5u(msg->txt);
	if (!strcmp(msg->label,"SA")) process_sa(msg->txt);
	if (!strcmp(msg->label,"Q1")) process_q1(msg->txt);
	if (!strcmp(msg->label,"Q2")) process_q2(msg->txt);
	if (!strcmp(msg->label,"QA")) process_qa(msg->txt);
	if (!strcmp(msg->label,"QB")) process_qb(msg->txt);
	if (!strcmp(msg->label,"QC")) process_qc(msg->txt);
	if (!strcmp(msg->label,"QD")) process_qd(msg->txt);
	if (!strcmp(msg->label,"QE")) process_qr(msg->txt);
	if (!strcmp(msg->label,"QF")) process_qf(msg->txt);
	if (!strcmp(msg->label,"QG")) process_qg(msg->txt);
	if (!strcmp(msg->label,"QH")) process_qh(msg->txt);
	if (!strcmp(msg->label,"QK")) process_qk(msg->txt);
	if (!strcmp(msg->label,"QL")) process_ql(msg->txt);
	if (!strcmp(msg->label,"QM")) process_qm(msg->txt);
	if (!strcmp(msg->label,"QN")) process_qn(msg->txt);
	if (!strcmp(msg->label,"QP")) process_qp(msg->txt);
	if (!strcmp(msg->label,"QQ")) process_qq(msg->txt);
	if (!strcmp(msg->label,"QR")) process_qr(msg->txt);
	if (!strcmp(msg->label,"QS")) process_qs(msg->txt);
	if (!strcmp(msg->label,"QT")) process_qt(msg->txt);
	if (!strcmp(msg->label,"57")) process_57(msg->txt);
	if (!strcmp(msg->label,"H1")) process_h1(msg->txt);
	if (!strcmp(msg->label,"54")) process_54(msg->txt);

	printf("Message content:-\n%s", msg->txt);



	rx_idx++;

	printf
	    ("\n\n[END_MESSAGE ]------------------------------------------------------------\n\n");

}

///////// END OF ACARS ROUTINES //////////






/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

void rotate_90(unsigned char *buf, uint32_t len)
/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
   or [0, 1, -3, 2, -4, -5, 7, -6] */
{
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		/* uint8_t negation = 255 - x */
		tmp = 255 - buf[i+3];
		buf[i+3] = buf[i+2];
		buf[i+2] = tmp;

		buf[i+4] = 255 - buf[i+4];
		buf[i+5] = 255 - buf[i+5];

		tmp = 255 - buf[i+6];
		buf[i+6] = buf[i+7];
		buf[i+7] = tmp;
	}
}

void low_pass(struct fm_state *fm, unsigned char *buf, uint32_t len)
/* simple square window FIR */
{
	int i=0, i2=0, seq=0;
	while (i < (int)len) {
		fm->now_r += ((int)buf[i]   - 127);
		fm->now_j += ((int)buf[i+1] - 127);
		i += 2;
		fm->prev_index++;

		if ( (fm->prev_index<(fm->downsample)) ) continue;
		if ((seq%2)==1)
		{
		// signal is ~10khz wide, don't need whole 48khz
		// eliminate some RF noise by attenuating stuff outside 24khz a bit
		fm->signal[i2]   = (fm->now_r*5)/(8); 
		fm->signal[i2+1] = (fm->now_j*5)/(8);
		}
		else
		{
		fm->signal[i2]   = fm->now_r;// * fm->output_scale;
		fm->signal[i2+1] = fm->now_j;// * fm->output_scale;
		}
		seq++;
		fm->prev_index = 0;
		fm->now_r = 0;
		fm->now_j = 0;
		i2 += 2;
	}
	fm->signal_len = i2;
}







void build_fir(struct fm_state *fm)
/* hamming */
/* point = sum(sample[i] * fir[i] * fir_len / fir_sum) */
{
	double a, b, w, N1;
	int i, len;
	len = fm->downsample;
	a = 25.0/46.0;
	b = 21.0/46.0;
	N1 = (double)(len-1);
	for(i = 0; i < len; i++) {
		w = a - b*cos(2*i*M_PI/N1);
		fm->fir[i] = (int)(w * 255);
	}
	fm->fir_sum = 0;
	for(i = 0; i < len; i++) {
		fm->fir_sum += fm->fir[i];
	}
}

void low_pass_fir(struct fm_state *fm, unsigned char *buf, uint32_t len)
/* perform an arbitrary FIR, doubles CPU use */
// possibly bugged, or overflowing
{
	int i=0, i2=0, i3=0;
	while (i < (int)len) {
		i3 = fm->prev_index;
		fm->now_r += ((int)buf[i]   - 127) * fm->fir[i3];
		fm->now_j += ((int)buf[i+1] - 127) * fm->fir[i3];
		i += 2;
		fm->prev_index++;
		
		if (fm->prev_index < fm->downsample) {
			continue;
		}
		fm->now_r *= fm->downsample;
		fm->now_j *= fm->downsample;
		fm->now_r /= fm->fir_sum;
		fm->now_j /= fm->fir_sum;
		fm->signal[i2]   = fm->now_r; //* fm->output_scale;
		fm->signal[i2+1] = fm->now_j; //* fm->output_scale;
		fm->prev_index = 0;
		fm->now_r = 0;
		fm->now_j = 0;
		i2 += 2;
	}
	fm->signal_len = i2;
}

int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		//signal2[i/step] = (int16_t)(sum / step);
		signal2[i/step] = (int16_t)(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

void low_pass_real(struct fm_state *fm)
/* simple square window FIR */
// add support for upsampling?
{
	int i=0, i2=0;
	int fast = (int)fm->sample_rate / (fm->post_downsample);
	int slow = fm->output_rate;
	while (i < fm->signal2_len) {
		fm->now_lpr += fm->signal2[i];
		i++;
		fm->prev_lpr_index += slow;
		if (fm->prev_lpr_index < fast) {
			continue;
		}
		fm->signal2[i2] = (int16_t)(fm->now_lpr / (fast/slow));
		fm->prev_lpr_index -= fast;
		fm->now_lpr = 0;
		i2 += 1;
	}
	fm->signal2_len = i2;
}

/* define our own complex math ops
   because ARMv5 has no hardware float */

void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

int polar_discriminant(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2((double)cj, (double)cr);
	return (int)(angle / 3.14159 * (1<<14));
}

int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	if (x >= 0) {
		angle = pi4  - pi4 * (x-yabs) / (x+yabs);
	} else {
		angle = pi34 - pi4 * (x+yabs) / (yabs-x);
	}
	if (y < 0) {
		return -angle;
	}
	return angle;
}

int polar_disc_fast(int ar, int aj, int br, int bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

int atan_lut_init()
{
	int i = 0;

	atan_lut = malloc(atan_lut_size * sizeof(int));

	for (i = 0; i < atan_lut_size; i++) {
		atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

int polar_disc_lut(int ar, int aj, int br, int bj)
{
	int cr, cj, x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = (cj << atan_lut_coef) / cr;
	x_abs = abs(x);

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -1<<13;
	}

	if (x > 0) {
		return (cj > 0) ? atan_lut[x] : atan_lut[x] - (1<<14);
	} else {
		return (cj > 0) ? (1<<14) - atan_lut[-x] : -atan_lut[-x];
	}

	return 0;
}


void am_demod(struct fm_state *fm)
// todo, fix this extreme laziness
{
	int i, pcm;
	for (i = 0; i < (fm->signal_len); i += 2) {
		// hypot uses floats but won't overflow
		//fm->signal2[i/2] = (int16_t)hypot(fm->signal[i], fm->signal[i+1]);
		pcm = fm->signal[i] * fm->signal[i];
		pcm += fm->signal[i+1] * fm->signal[i+1];
		fm->signal2[i/2] = (int16_t)sqrt(pcm) * fm->output_scale;
		// Milen: add some gain to signal
		fm->signal2[i/2] *= 8;
	}
	fm->signal2_len = fm->signal_len/2;
	// lowpass? (3khz)  highpass?  (dc)
}



void deemph_filter(struct fm_state *fm)
{
	static int avg;  // cheating...
	int i, d;
	// de-emph IIR
	// avg = avg * (1 - alpha) + sample * alpha;
	for (i = 0; i < fm->signal2_len; i++) {
		d = fm->signal2[i] - avg;
		if (d > 0) {
			avg += (d + fm->deemph_a/2) / fm->deemph_a;
		} else {
			avg += (d - fm->deemph_a/2) / fm->deemph_a;
		}
		fm->signal2[i] = (int16_t)avg;
	}
}

void dc_block_filter(struct fm_state *fm)
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < fm->signal2_len; i++) {
		sum += fm->signal2[i];
	}
	avg = sum / fm->signal2_len;
	avg = (avg + fm->dc_avg * 9) / 10;
	for (i=0; i < fm->signal2_len; i++) {
		fm->signal2[i] -= avg;
	}
	fm->dc_avg = avg;
}

int mad(int *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int post_squelch(struct fm_state *fm)
/* returns 1 for active signal, 0 for no signal */
{
	int dev_r, dev_j, len, sq_l;
	/* only for small samples, big samples need chunk processing */
	len = fm->signal_len;
	sq_l = fm->squelch_level;
	dev_r = mad(&(fm->signal[0]), len, 2);
	dev_j = mad(&(fm->signal[1]), len, 2);
//fprintf(stderr,"shits: %d dr: %d dj: %d sql: %d\n ",fm->squelch_hits,dev_r,dev_j,sq_l);

	if ((dev_r > sq_l) || (dev_j > sq_l)) {
		fm->squelch_hits = 0;
		return 1;
	}
	fm->squelch_hits++;
	return 0;
}

static void optimal_settings(struct fm_state *fm, int freq, int hopping)
{
	int r, capture_freq, capture_rate;
	fm->downsample = (1000000 / fm->sample_rate) + 1;

	fm->freq_now = freq;
	capture_rate = fm->downsample * fm->sample_rate;
	capture_freq = fm->freqs[freq] + capture_rate/4;
	capture_freq += fm->edge * fm->sample_rate / 2;
	fm->output_scale = (1<<15) / (128 * fm->downsample);
	if (fm->output_scale < 1) {
		fm->output_scale = 1;}
	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, (uint32_t)capture_freq);
	if (hopping) {
		return;}
		
	// Milen: don't need 48khz signal, set cutoff at ~16khz
	//fm->downsample /=2;
	//fm->post_downsample*=2;

		
	fprintf(stderr, "Oversampling input by: %ix.\n", fm->downsample);
	fprintf(stderr, "Oversampling output by: %ix.\n", fm->post_downsample);
	fprintf(stderr, "Buffer size: %0.2fms\n",
		1000 * 0.5 * (float)ACTUAL_BUF_LENGTH / (float)capture_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");}
	else {
		fprintf(stderr, "Tuned to %u Hz.\n", capture_freq);}

	/* Set the sample rate */
	fprintf(stderr, "Sampling at %u Hz.\n", capture_rate);
	if (fm->output_rate > 0) {
		fprintf(stderr, "Output at %u Hz.\n", fm->output_rate);
	} else {
		fprintf(stderr, "Output at %u Hz.\n", fm->sample_rate/fm->post_downsample);}
	r = rtlsdr_set_sample_rate(dev, (uint32_t)capture_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");}

}

void full_demod(struct fm_state *fm)
{
	uint8_t dump[BUFFER_DUMP];
	int i, sr, freq_next, n_read, hop = 0;
	pthread_rwlock_wrlock(&data_rw);
	rotate_90(fm->buf, fm->buf_len);
	if (fm->fir_enable) {
		low_pass_fir(fm, fm->buf, fm->buf_len);
	} else {
		low_pass(fm, fm->buf, fm->buf_len);
	}
	pthread_rwlock_unlock(&data_rw);

	sr = post_squelch(fm);
	if (!sr && fm->squelch_hits > 1/*fm->conseq_squelch*/) {
		//if (fm->terminate_on_squelch) {
		//	fm->exit_flag = 1;}
		if (fm->freq_len == 1) {  /* mute */
			for (i=0; i<fm->signal_len; i++) {
				fm->signal2[i] = 0;}
		}
		else {
			hop = 1;
		     }
	}
	if (fm->post_downsample > 1) {
		fm->signal2_len = low_pass_simple(fm->signal2, fm->signal2_len, fm->post_downsample);}
	if (fm->output_rate > 0) {
		low_pass_real(fm);
	}
	if (fm->deemph) {
		deemph_filter(fm);}
	if (fm->dc_block) {
		dc_block_filter(fm);}
	/* ignore under runs for now */

	//fwrite(fm->signal2, 2, fm->signal2_len, fm->file);
	if (hop) {
		if (debug_hop) fprintf(stderr,"Hopping freq!\n");
		freq_next = (fm->freq_now + 1) % fm->freq_len;
		optimal_settings(fm, freq_next, 1);
		current_freq = fm->freqs[freq_next];
		fm->squelch_hits = fm->conseq_squelch + 1;  /* hair trigger */
		/* wait for settling and flush buffer */
		//usleep(5000);
		usleep(1000);
		rtlsdr_read_sync(dev, &dump, BUFFER_DUMP, &n_read);
		if (n_read != BUFFER_DUMP) {
			fprintf(stderr, "Error: bad retune.\n");}
	}
	else am_demod(fm);

}


void acars_decode(struct fm_state *fm) {
	int16_t* sample = fm->signal2;
	int ind;	
	for (ind = 0; ind < fm->signal2_len;) {
		nbitl += getbit(sample[ind], &rl, 0);
		if (nbitl >= nrbitl) {
			nrbitl = getmesg(rl, &msgl, 0);
			nbitl = 0;
			if (nrbitl == 0) {
				print_mesg(&msgl);
				nrbitl = 8;
			}
		}
		ind++;
	}
}


static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct fm_state *fm2 = ctx;
	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	pthread_rwlock_wrlock(&data_rw);
	memcpy(fm2->buf, buf, len);
	fm2->buf_len = len;
	pthread_rwlock_unlock(&data_rw);
	safe_cond_signal(&data_ready, &data_mutex);
	/* single threaded uses 25% less CPU? */
	/* full_demod(fm2); */
}

static void sync_read(unsigned char *buf, uint32_t len, struct fm_state *fm)
{
	int r, n_read;
	r = rtlsdr_read_sync(dev, buf, len, &n_read);
	if (r < 0) {
		fprintf(stderr, "WARNING: sync read failed.\n");
		return;
	}
	pthread_rwlock_wrlock(&data_rw);
	memcpy(fm->buf, buf, len);
	fm->buf_len = len;
	pthread_rwlock_unlock(&data_rw);
	safe_cond_signal(&data_ready, &data_mutex);
	//full_demod(fm);
}

static void *demod_thread_fn(void *arg)
{
	struct fm_state *fm2 = arg;
	// So that DBs will be loaded, we'd better use a mutex here
	pthread_mutex_lock(&dataset_mutex);
	pthread_mutex_unlock(&dataset_mutex);

	while (!do_exit) {
		safe_cond_wait(&data_ready, &data_mutex);
		full_demod(fm2);
		acars_decode(fm2);
		if (fm2->exit_flag) {
			do_exit = 1;
			//rtlsdr_cancel_async(dev);
		}
	}
	return 0;
}

double atofs(char *f)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(f);
	last = f[len-1];
	f[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
		case 'm':
		case 'M':
			suff *= 1e3;
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(f);
			f[len-1] = last;
			return suff;
	}
	f[len-1] = last;
	return atof(f);
}

void frequency_range(struct fm_state *fm, char *arg)
{
	char *start, *stop, *step;
	int i;
	start = arg;
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
	{
		fm->freqs[fm->freq_len] = (uint32_t)i;
		fm->freq_len++;
		if (fm->freq_len >= FREQUENCIES_LIMIT) {
			break;}
	}
	stop[-1] = ':';
	step[-1] = ':';
}

int nearest_gain(int target_gain)
{
	int i, err1, err2, count, close_gain;
	int* gains;
	count = rtlsdr_get_tuner_gains(dev, NULL);
	if (count <= 0) {
		return 0;
	}
	gains = malloc(sizeof(int) * count);
	count = rtlsdr_get_tuner_gains(dev, gains);
	close_gain = gains[0];
	for (i=0; i<count; i++) {
		err1 = abs(target_gain - close_gain);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			close_gain = gains[i];
		}
	}
	free(gains);
	return close_gain;
}

void fm_init(struct fm_state *fm)
{
	fm->freqs[0] = 100000000;
	fm->sample_rate = 48000;
	fm->squelch_level = 0;
	fm->conseq_squelch = 0;
	fm->terminate_on_squelch = 0;
	fm->squelch_hits = 0;
	fm->freq_len = 0;
	fm->edge = 0;
	fm->fir_enable = 0;
	fm->prev_index = 0;
	fm->post_downsample = 1;  // once this works, default = 4
	fm->custom_atan = 0;
	fm->deemph = 0;
	fm->output_rate = -1;  // flag for disabled
	fm->mode_demod = &am_demod;
	fm->pre_j = fm->pre_r = fm->now_r = fm->now_j = 0;
	fm->prev_lpr_index = 0;
	fm->deemph_a = 0;
	fm->now_lpr = 0;
	fm->dc_block = 0;
	fm->dc_avg = 0;

	fm->sample_rate = 48000;
	fm->freq_len = 0;
	fm->edge = 0;
	fm->prev_index = 0;
	fm->post_downsample = 1;  // once this works, default = 4
	fm->output_rate = -1; // disabled
	fm->pre_j = fm->pre_r = fm->now_r = fm->now_j = 0;
	fm->prev_lpr_index = 0;
	fm->now_lpr = 0;
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	struct fm_state fm; 
	char *filename = NULL;
	int n_read, r, opt, wb_mode = 0;
	int i, gain = AUTO_GAIN; // tenths of a dB
	uint8_t *buffer;
	uint32_t dev_index = 0;
	int device_count;
	int ppm_error = 0;
	char vendor[256], product[256], serial[256];
	fm_init(&fm);
	pthread_cond_init(&data_ready, NULL);
	pthread_rwlock_init(&data_rw, NULL);
	pthread_mutex_init(&data_mutex, NULL);
	pthread_mutex_init(&dataset_mutex, NULL);
	fm.sample_rate = (uint32_t) 48000;

	while ((opt = getopt(argc, argv, "d:f:g:s:b:l:o:t:p:EFA:rNWMULRDCh")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = atoi(optarg);
			break;
		case 'f':
			if (fm.freq_len >= FREQUENCIES_LIMIT) {
				break;}
			if (strchr(optarg, ':'))
				{frequency_range(&fm, optarg);}
			else
			{
				fm.freqs[fm.freq_len] = (uint32_t)atofs(optarg);
				fm.freq_len++;
			}
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10);
			break;
		case 'l':
			fm.squelch_level = (int)atof(optarg);
			break;
		case 'o':
			fm.post_downsample = (int)atof(optarg);
			if (fm.post_downsample < 1 || fm.post_downsample > MAXIMUM_OVERSAMPLE) {
				fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);}
			break;
		case 't':
			fm.conseq_squelch = (int)atof(optarg);
			if (fm.conseq_squelch < 0) {
				fm.conseq_squelch = -fm.conseq_squelch;
				fm.terminate_on_squelch = 1;
			}
			break;
		case 'r':
			debug_hop = 1;
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'F':
			fm.fir_enable = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}
	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	fm.sample_rate *= fm.post_downsample;

	if (fm.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}

	if (fm.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

	if (fm.freq_len > 1 && fm.squelch_level == 0) {
		fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
		exit(1);
	}

	if (fm.freq_len > 1) {
		fm.terminate_on_squelch = 0;
	}

	if (argc <= optind) {
		filename = "-";
	} else {
		filename = argv[optind];
	}

	ACTUAL_BUF_LENGTH = lcm_post[fm.post_downsample] * DEFAULT_BUF_LENGTH;
	buffer = malloc(ACTUAL_BUF_LENGTH * sizeof(uint8_t));

	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		exit(1);
	}

	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "Using device %d: %s\n",
		dev_index, rtlsdr_get_device_name(dev_index));

	r = rtlsdr_open(&dev, dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* WBFM is special */
	// I really should loop over everything
	// but you are more wrong for scanning broadcast FM
	if (wb_mode) {
		fm.freqs[0] += 16000;
	}

	if (fm.deemph) {
		fm.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(fm.output_rate * 75e-6)))));
	}

	optimal_settings(&fm, 0, 0);
	build_fir(&fm);

	/* Set the tuner gain */
	if (gain == AUTO_GAIN) {
		r = rtlsdr_set_tuner_gain_mode(dev, 0);
	} else {
		r = rtlsdr_set_tuner_gain_mode(dev, 1);
		gain = nearest_gain(gain);
		r = rtlsdr_set_tuner_gain(dev, gain);
	}
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else if (gain == AUTO_GAIN) {
		fprintf(stderr, "Tuner gain set to automatic.\n");
	} else {
		fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
	}
	r = rtlsdr_set_freq_correction(dev, ppm_error);

	if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
		fm.file = stdout;
#ifdef _WIN32
		//_setmode(_fileno(fm.file), _O_BINARY);
#endif
	} else {
		/* DO NOT WRITE TO FILE 
		
		fm.file = fopen(filename, "wb");
		if (!fm.file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			exit(1);
		}
		*/
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");}
	pthread_mutex_lock(&dataset_mutex);
	pthread_create(&demod_thread, NULL, demod_thread_fn, (void *)(&fm));
	/*rtlsdr_read_async(dev, rtlsdr_callback, (void *)(&fm),
			      DEFAULT_ASYNC_BUF_NUMBER,
			      ACTUAL_BUF_LENGTH);*/
	fprintf(stderr, "\n");
	load_aircrafts();
	load_airports();
	load_flights();
	load_message_labels();
	init_bits();
	init_mesg();
	printf("Listening for ACARS traffic...\n");
	fprintf(stderr, "\n");
	pthread_mutex_unlock(&dataset_mutex);

	while (!do_exit) {
		sync_read(buffer, ACTUAL_BUF_LENGTH, &fm);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}



	//rtlsdr_cancel_async(dev);
	safe_cond_signal(&data_ready, &data_mutex);
	pthread_join(demod_thread, NULL);

	pthread_cond_destroy(&data_ready);
	pthread_rwlock_destroy(&data_rw);
	pthread_mutex_destroy(&data_mutex);

	/*
	if (fm.file != stdout) {
		fclose(fm.file);}
	*/
	rtlsdr_close(dev);
	free (buffer);
	return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
