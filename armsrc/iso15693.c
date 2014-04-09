//-----------------------------------------------------------------------------
// Jonathan Westhues, split Nov 2006
// Modified by Greg Jones, Jan 2009
// Modified by Adrian Dabrowski "atrox", Mar-Sept 2010,Oct 2011
//
// This code is licensed to you under the terms of the GNU GPL, version 2 or,
// at your option, any later version. See the LICENSE.txt file for the text of
// the license.
//-----------------------------------------------------------------------------
// Routines to support ISO 15693. This includes both the reader software and
// the `fake tag' modes, but at the moment I've implemented only the reader
// stuff, and that barely.
// Modified to perform modulation onboard in arm rather than on PC
// Also added additional reader commands (SELECT, READ etc.)
//-----------------------------------------------------------------------------
// The ISO 15693 describes two transmission modes from reader to tag, and 4 
// transmission modes from tag to reader. As of Mar 2010 this code only 
// supports one of each: "1of4" mode from reader to tag, and the highspeed 
// variant with one subcarrier from card to reader.
// As long, as the card fully support ISO 15693 this is no problem, since the 
// reader chooses both data rates, but some non-standard tags do not. Further for 
// the simulation to work, we will need to support all data rates.
//
// VCD (reader) -> VICC (tag)
// 1 out of 256:
// 	data rate: 1,66 kbit/s (fc/8192) 
// 	used for long range
// 1 out of 4:
// 	data rate: 26,48 kbit/s (fc/512)
//	used for short range, high speed
// 
// VICC (tag) -> VCD (reader)
// Modulation:
//		ASK / one subcarrier (423,75 khz)
//		FSK / two subcarriers (423,75 khz && 484,28 khz)
// Data Rates / Modes:
// 	low ASK: 6,62 kbit/s
// 	low FSK: 6.67 kbit/s
// 	high ASK: 26,48 kbit/s
// 	high FSK: 26,69 kbit/s
//-----------------------------------------------------------------------------
// added "1 out of 256" mode (for VCD->PICC) - atrox 20100911


// Random Remarks:
// *) UID is always used "transmission order" (LSB), which is reverse to display order

// TODO / BUGS / ISSUES:
// *) writing to tags takes longer: we miss the answer from the tag in most cases
//    -> tweak the read-timeout times
// *) signal decoding from the card is still a bit shaky. 
// *) signal decoding is unable to detect collissions.
// *) add anti-collission support for inventory-commands 
// *) read security status of a block
// *) sniffing and simulation do only support one transmission mode. need to support 
//		all 8 transmission combinations
//	*) remove or refactor code under "depricated"
// *) document all the functions


#include "proxmark3.h"
#include "util.h"
#include "apps.h"
#include "string.h"
#include "iso15693tools.h"
#include "cmd.h"

#define arraylen(x) (sizeof(x)/sizeof((x)[0]))

///////////////////////////////////////////////////////////////////////
// ISO 15693 Part 2 - Air Interface
// This section basicly contains transmission and receiving of bits
///////////////////////////////////////////////////////////////////////

#define FrameSOF              Iso15693FrameSOF
#define Logic0                Iso15693Logic0
#define Logic1                Iso15693Logic1
#define FrameEOF              Iso15693FrameEOF

#define Crc(data,datalen)     Iso15693Crc(data,datalen)
#define AddCrc(data,datalen)  Iso15693AddCrc(data,datalen)
#define sprintUID(target,uid)	Iso15693sprintUID(target,uid)

int DEBUG=0;

//-----------------------------------------------------------------------------
// The software UART that receives commands from the reader, and its state
// variables.
//-----------------------------------------------------------------------------
static struct {
    enum {
        STATE_UNSYNCD,
        STATE_START_OF_COMMUNICATION,
	STATE_RECEIVING
    }       state;
    uint16_t    shiftReg;
    int     bitCnt;
    int     byteCnt;
    int     byteCntMax;
    int     posCnt;
    int     nOutOfCnt;
    int     OutOfCnt;
    int     syncBit;
    int     parityBits;
    int     samples;
    int     highCnt;
    int     swapper;
    int     counter;
    int     bitBuffer;
    int     dropPosition;
    uint8_t   *output;
} Uart;

static RAMFUNC int OutOfNDecoding(int bit)
{
	//int error = 0;
	int bitright;

	if(!Uart.bitBuffer) {
		Uart.bitBuffer = bit ^ 0xFF0;
		return FALSE;
	}
	else {
		Uart.bitBuffer <<= 4;
		Uart.bitBuffer ^= bit;
	}
	
	/*if(Uart.swapper) {
		Uart.output[Uart.byteCnt] = Uart.bitBuffer & 0xFF;
		Uart.byteCnt++;
		Uart.swapper = 0;
		if(Uart.byteCnt > 15) { return TRUE; }
	}
	else {
		Uart.swapper = 1;
	}*/

	if(Uart.state != STATE_UNSYNCD) {
		Uart.posCnt++;

		if((Uart.bitBuffer & Uart.syncBit) ^ Uart.syncBit) {
			bit = 0x00;
		}
		else {
			bit = 0x01;
		}
		if(((Uart.bitBuffer << 1) & Uart.syncBit) ^ Uart.syncBit) {
			bitright = 0x00;
		}
		else {
			bitright = 0x01;
		}
		if(bit != bitright) { bit = bitright; }

		
		// So, now we only have to deal with *bit*, lets see...
		if(Uart.posCnt == 1) {
			// measurement first half bitperiod
			if(!bit) {
				// Drop in first half means that we are either seeing
				// an SOF or an EOF.

				if(Uart.nOutOfCnt == 1) {
					// End of Communication
					Uart.state = STATE_UNSYNCD;
					Uart.highCnt = 0;
					if(Uart.byteCnt == 0) {
						// Its not straightforward to show single EOFs
						// So just leave it and do not return TRUE
						Uart.output[Uart.byteCnt] = 0xf0;
						Uart.byteCnt++;

						// Calculate the parity bit for the client...
						Uart.parityBits = 1;
					}
					else {
						return TRUE;
					}
				}
				else if(Uart.state != STATE_START_OF_COMMUNICATION) {
					// When not part of SOF or EOF, it is an error
					Uart.state = STATE_UNSYNCD;
					Uart.highCnt = 0;
					//error = 4;
				}
			}
		}
		else {
			// measurement second half bitperiod
			// Count the bitslot we are in... (ISO 15693)
			Uart.nOutOfCnt++;
			
			if(!bit) {
				if(Uart.dropPosition) {
					if(Uart.state == STATE_START_OF_COMMUNICATION) {
						//error = 1;
					}
					else {
						//error = 7;
					}
					// It is an error if we already have seen a drop in current frame
					Uart.state = STATE_UNSYNCD;
					Uart.highCnt = 0;
				}
				else {
					Uart.dropPosition = Uart.nOutOfCnt;
				}
			}

			Uart.posCnt = 0;

			
			if(Uart.nOutOfCnt == Uart.OutOfCnt && Uart.OutOfCnt == 4) {
				Uart.nOutOfCnt = 0;
				
				if(Uart.state == STATE_START_OF_COMMUNICATION) {
					if(Uart.dropPosition == 4) {
						Uart.state = STATE_RECEIVING;
						Uart.OutOfCnt = 256;
					}
					else if(Uart.dropPosition == 3) {
						Uart.state = STATE_RECEIVING;
						Uart.OutOfCnt = 4;
						//Uart.output[Uart.byteCnt] = 0xdd;
						//Uart.byteCnt++;
					}
					else {
						Uart.state = STATE_UNSYNCD;
						Uart.highCnt = 0;
					}
					Uart.dropPosition = 0;
				}
				else {
					// RECEIVING DATA
					// 1 out of 4
					if(!Uart.dropPosition) {
						Uart.state = STATE_UNSYNCD;
						Uart.highCnt = 0;
						//error = 9;
					}
					else {
						Uart.shiftReg >>= 2;
						
						// Swap bit order
						Uart.dropPosition--;
						//if(Uart.dropPosition == 1) { Uart.dropPosition = 2; }
						//else if(Uart.dropPosition == 2) { Uart.dropPosition = 1; }
						
						Uart.shiftReg ^= ((Uart.dropPosition & 0x03) << 6);
						Uart.bitCnt += 2;
						Uart.dropPosition = 0;

						if(Uart.bitCnt == 8) {
							Uart.output[Uart.byteCnt] = (Uart.shiftReg & 0xff);
							Uart.byteCnt++;

							// Calculate the parity bit for the client...
							Uart.parityBits <<= 1;
							Uart.parityBits ^= OddByteParity[(Uart.shiftReg & 0xff)];

							Uart.bitCnt = 0;
							Uart.shiftReg = 0;
						}
					}
				}
			}
			else if(Uart.nOutOfCnt == Uart.OutOfCnt) {
				// RECEIVING DATA
				// 1 out of 256
				if(!Uart.dropPosition) {
					Uart.state = STATE_UNSYNCD;
					Uart.highCnt = 0;
					//error = 3;
				}
				else {
					Uart.dropPosition--;
					Uart.output[Uart.byteCnt] = (Uart.dropPosition & 0xff);
					Uart.byteCnt++;

					// Calculate the parity bit for the client...
					Uart.parityBits <<= 1;
					Uart.parityBits ^= OddByteParity[(Uart.dropPosition & 0xff)];

					Uart.bitCnt = 0;
					Uart.shiftReg = 0;
					Uart.nOutOfCnt = 0;
					Uart.dropPosition = 0;
				}
			}

			/*if(error) {
				Uart.output[Uart.byteCnt] = 0xAA;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = error & 0xFF;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = 0xAA;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = (Uart.bitBuffer >> 8) & 0xFF;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = Uart.bitBuffer & 0xFF;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = (Uart.syncBit >> 3) & 0xFF;
				Uart.byteCnt++;
				Uart.output[Uart.byteCnt] = 0xAA;
				Uart.byteCnt++;
				return TRUE;
			}*/
		}

	}
	else {
		bit = Uart.bitBuffer & 0xf0;
		bit >>= 4;
		bit ^= 0x0F; // drops become 1s ;-)
		if(bit) {
			// should have been high or at least (4 * 128) / fc
			// according to ISO this should be at least (9 * 128 + 20) / fc
			if(Uart.highCnt == 8) {
				// we went low, so this could be start of communication
				// it turns out to be safer to choose a less significant
				// syncbit... so we check whether the neighbour also represents the drop
				Uart.posCnt = 1;   // apparently we are busy with our first half bit period
				Uart.syncBit = bit & 8;
				Uart.samples = 3;
				if(!Uart.syncBit)	{ Uart.syncBit = bit & 4; Uart.samples = 2; }
				else if(bit & 4)	{ Uart.syncBit = bit & 4; Uart.samples = 2; bit <<= 2; }
				if(!Uart.syncBit)	{ Uart.syncBit = bit & 2; Uart.samples = 1; }
				else if(bit & 2)	{ Uart.syncBit = bit & 2; Uart.samples = 1; bit <<= 1; }
				if(!Uart.syncBit)	{ Uart.syncBit = bit & 1; Uart.samples = 0;
					if(Uart.syncBit && (Uart.bitBuffer & 8)) {
						Uart.syncBit = 8;

						// the first half bit period is expected in next sample
						Uart.posCnt = 0;
						Uart.samples = 3;
					}
				}
				else if(bit & 1)	{ Uart.syncBit = bit & 1; Uart.samples = 0; }

				Uart.syncBit <<= 4;
				Uart.state = STATE_START_OF_COMMUNICATION;
				Uart.bitCnt = 0;
				Uart.byteCnt = 0;
				Uart.parityBits = 0;
				Uart.nOutOfCnt = 0;
				Uart.OutOfCnt = 4; // Start at 1/4, could switch to 1/256
				Uart.dropPosition = 0;
				Uart.shiftReg = 0;
				//error = 0;
			}
			else {
				Uart.highCnt = 0;
			}
		}
		else {
			if(Uart.highCnt < 8) {
				Uart.highCnt++;
			}
		}
	}

    return FALSE;
}


// ---------------------------
// Signal Processing 
// ---------------------------

// prepare data using "1 out of 4" code for later transmission
// resulting data rate is 26,48 kbit/s (fc/512)
// cmd ... data
// n ... length of data
static void CodeIso15693AsReader(uint8_t *cmd, int n)
{
	int i, j;

	ToSendReset();

	// Give it a bit of slack at the beginning
	for(i = 0; i < 24; i++) {
		ToSendStuffBit(1);
	}

	// SOF for 1of4
	ToSendStuffBit(0);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(0);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	for(i = 0; i < n; i++) {
		for(j = 0; j < 8; j += 2) {
			int these = (cmd[i] >> j) & 3;
			switch(these) {
				case 0:
					ToSendStuffBit(1);
					ToSendStuffBit(0);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					break;
				case 1:
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(0);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					break;
				case 2:
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(0);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					break;
				case 3:
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(1);
					ToSendStuffBit(0);
					break;
			}
		}
	}
	// EOF
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(0);
	ToSendStuffBit(1);

	// And slack at the end, too.
	for(i = 0; i < 24; i++) {
		ToSendStuffBit(1);
	}
}

// encode data using "1 out of 256" sheme
// data rate is 1,66 kbit/s (fc/8192) 
// is designed for more robust communication over longer distances
static void CodeIso15693AsReader256(uint8_t *cmd, int n)
{
	int i, j;

	ToSendReset();

	// Give it a bit of slack at the beginning
	for(i = 0; i < 24; i++) {
		ToSendStuffBit(1);
	}

	// SOF for 1of256
	ToSendStuffBit(0);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(0);
	
	for(i = 0; i < n; i++) {
		for (j = 0; j<=255; j++) {
			if (cmd[i]==j) {
				ToSendStuffBit(1);
				ToSendStuffBit(0);
			} else {
				ToSendStuffBit(1);
				ToSendStuffBit(1);
			}			
		}	
	}
	// EOF
	ToSendStuffBit(1);
	ToSendStuffBit(1);
	ToSendStuffBit(0);
	ToSendStuffBit(1);

	// And slack at the end, too.
	for(i = 0; i < 24; i++) {
		ToSendStuffBit(1);
	}
}


// Transmit the command (to the tag) that was placed in ToSend[].
static void TransmitTo15693Tag(const uint8_t *cmd, int len, int *samples, int *wait)
{
    int c;

//    FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_ISO14443A_READER_MOD);
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_TX);
	if(*wait < 10) { *wait = 10; }

//    for(c = 0; c < *wait;) {
//        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
//            AT91C_BASE_SSC->SSC_THR = 0x00;		// For exact timing!
//            c++;
//        }
//        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
//            volatile uint32_t r = AT91C_BASE_SSC->SSC_RHR;
//            (void)r;
//        }
//        WDT_HIT();
//    }

    c = 0;
    for(;;) {
        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
            AT91C_BASE_SSC->SSC_THR = cmd[c];
            c++;
            if(c >= len) {
                break;
            }
        }
        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
            volatile uint32_t r = AT91C_BASE_SSC->SSC_RHR;
            (void)r;
        }
        WDT_HIT();
    }
	*samples = (c + *wait) << 3;
}

// //-----------------------------------------------------------------------------
// // Transmit the command (to the reader) that was placed in ToSend[].
// //-----------------------------------------------------------------------------
// static void TransmitTo15693Reader(const uint8_t *cmd, int len, int *samples, int *wait)
// {
//     int c;

// //	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_TX);
// 	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_SIMULATOR);	// No requirement to energise my coils
// 	if(*wait < 10) { *wait = 10; }

//     c = 0;
//     for(;;) {
//         if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
//             AT91C_BASE_SSC->SSC_THR = cmd[c];
//             c++;
//             if(c >= len) {
//                 break;
//             }
//         }
//         if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
//             volatile uint32_t r = AT91C_BASE_SSC->SSC_RHR;
//             (void)r;
//         }
//         WDT_HIT();
//     }
// 	*samples = (c + *wait) << 3;
// }


// Read from Tag
// Parameters:
//		receivedResponse
//		maxLen
//		samples
//		elapsed
// returns: 
//		number of decoded bytes
static int GetIso15693AnswerFromTag(uint8_t *receivedResponse, int maxLen, int *samples, int *elapsed)
{
	int c = 0;
	uint8_t *dest = (uint8_t *)BigBuf;
	int getNext = 0;

	int8_t prev = 0;

// NOW READ RESPONSE
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);
	//spindelay(60);	// greg - experiment to get rid of some of the 0 byte/failed reads
	c = 0;
	getNext = FALSE;
	for(;;) {
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
			AT91C_BASE_SSC->SSC_THR = 0x43;
		}
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
			int8_t b;
			b = (int8_t)AT91C_BASE_SSC->SSC_RHR;

			// The samples are correlations against I and Q versions of the
			// tone that the tag AM-modulates, so every other sample is I,
			// every other is Q. We just want power, so abs(I) + abs(Q) is
			// close to what we want.
			if(getNext) {
				int8_t r;

				if(b < 0) {
					r = -b;
				} else {
					r = b;
				}
				if(prev < 0) {
					r -= prev;
				} else {
					r += prev;
				}

				dest[c++] = (uint8_t)r;

				if(c >= 2000) {
					break;
				}
			} else {
				prev = b;
			}

			getNext = !getNext;
		}
	}

	//////////////////////////////////////////
	/////////// DEMODULATE ///////////////////
	//////////////////////////////////////////

	int i, j;
	int max = 0, maxPos=0;

	int skip = 4;

	//	if(GraphTraceLen < 1000) return;	// THIS CHECKS FOR A BUFFER TO SMALL

	// First, correlate for SOF
	for(i = 0; i < 100; i++) {
		int corr = 0;
		for(j = 0; j < arraylen(FrameSOF); j += skip) {
			corr += FrameSOF[j]*dest[i+(j/skip)];
		}
		if(corr > max) {
			max = corr;
			maxPos = i;
		}
	}
	//	DbpString("SOF at %d, correlation %d", maxPos,max/(arraylen(FrameSOF)/skip));

	int k = 0; // this will be our return value

	// greg - If correlation is less than 1 then there's little point in continuing
	if ((max/(arraylen(FrameSOF)/skip)) >= 1)
	{

		i = maxPos + arraylen(FrameSOF)/skip;
	
		uint8_t outBuf[20];
		memset(outBuf, 0, sizeof(outBuf));
		uint8_t mask = 0x01;
		for(;;) {
			int corr0 = 0, corr1 = 0, corrEOF = 0;
			for(j = 0; j < arraylen(Logic0); j += skip) {
				corr0 += Logic0[j]*dest[i+(j/skip)];
			}
			for(j = 0; j < arraylen(Logic1); j += skip) {
				corr1 += Logic1[j]*dest[i+(j/skip)];
			}
			for(j = 0; j < arraylen(FrameEOF); j += skip) {
				corrEOF += FrameEOF[j]*dest[i+(j/skip)];
			}
			// Even things out by the length of the target waveform.
			corr0 *= 4;
			corr1 *= 4;
	
			if(corrEOF > corr1 && corrEOF > corr0) {
	//			DbpString("EOF at %d", i);
				break;
			} else if(corr1 > corr0) {
				i += arraylen(Logic1)/skip;
				outBuf[k] |= mask;
			} else {
				i += arraylen(Logic0)/skip;
			}
			mask <<= 1;
			if(mask == 0) {
				k++;
				mask = 0x01;
			}
			if((i+(int)arraylen(FrameEOF)) >= 2000) {
				DbpString("ran off end!");
				break;
			}
		}
		if(mask != 0x01) { // this happens, when we miss the EOF
			// TODO: for some reason this happens quite often
			if (DEBUG) Dbprintf("error, uneven octet! (extra bits!) mask=%02x", mask);
			if (mask<0x08) k--; // discard the last uneven octet;
			// 0x08 is an assumption - but works quite often
		}
	//	uint8_t str1 [8];
	//	itoa(k,str1);
	//	strncat(str1," octets read",8);
	
	//	DbpString(  str1);    // DbpString("%d octets", k);
	
	//	for(i = 0; i < k; i+=3) {
	//		//DbpString("# %2d: %02x ", i, outBuf[i]);
	//		DbpIntegers(outBuf[i],outBuf[i+1],outBuf[i+2]);
	//	}
	
		for(i = 0; i < k; i++) {
			receivedResponse[i] = outBuf[i];
		}
	} // "end if correlation > 0" 	(max/(arraylen(FrameSOF)/skip))
	return k; // return the number of bytes demodulated

///	DbpString("CRC=%04x", Iso15693Crc(outBuf, k-2));

}


// // Now the GetISO15693 message from sniffing command
// static int GetIso15693AnswerFromSniff(uint8_t *receivedResponse, int maxLen, int *samples, int *elapsed)
// {
// 	int c = 0;
// 	uint8_t *dest = (uint8_t *)BigBuf;
// 	int getNext = 0;

// 	int8_t prev = 0;

// // NOW READ RESPONSE
// 	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);
// 	//spindelay(60);	// greg - experiment to get rid of some of the 0 byte/failed reads
// 	c = 0;
// 	getNext = FALSE;
// 	for(;;) {
// 		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
// 			AT91C_BASE_SSC->SSC_THR = 0x43;
// 		}
// 		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
// 			int8_t b;
// 			b = (int8_t)AT91C_BASE_SSC->SSC_RHR;

// 			// The samples are correlations against I and Q versions of the
// 			// tone that the tag AM-modulates, so every other sample is I,
// 			// every other is Q. We just want power, so abs(I) + abs(Q) is
// 			// close to what we want.
// 			if(getNext) {
// 				int8_t r;

// 				if(b < 0) {
// 					r = -b;
// 				} else {
// 					r = b;
// 				}
// 				if(prev < 0) {
// 					r -= prev;
// 				} else {
// 					r += prev;
// 				}

// 				dest[c++] = (uint8_t)r;

// 				if(c >= 20000) {
// 					break;
// 				}
// 			} else {
// 				prev = b;
// 			}

// 			getNext = !getNext;
// 		}
// 	}

// 	//////////////////////////////////////////
// 	/////////// DEMODULATE ///////////////////
// 	//////////////////////////////////////////

// 	int i, j;
// 	int max = 0, maxPos=0;

// 	int skip = 4;

// //	if(GraphTraceLen < 1000) return;	// THIS CHECKS FOR A BUFFER TO SMALL

// 	// First, correlate for SOF
// 	for(i = 0; i < 19000; i++) {
// 		int corr = 0;
// 		for(j = 0; j < arraylen(FrameSOF); j += skip) {
// 			corr += FrameSOF[j]*dest[i+(j/skip)];
// 		}
// 		if(corr > max) {
// 			max = corr;
// 			maxPos = i;
// 		}
// 	}
// //	DbpString("SOF at %d, correlation %d", maxPos,max/(arraylen(FrameSOF)/skip));

// 	int k = 0; // this will be our return value

// 	// greg - If correlation is less than 1 then there's little point in continuing
// 	if ((max/(arraylen(FrameSOF)/skip)) >= 1)	// THIS SHOULD BE 1
// 	{
	
// 		i = maxPos + arraylen(FrameSOF)/skip;
	
// 		uint8_t outBuf[20];
// 		memset(outBuf, 0, sizeof(outBuf));
// 		uint8_t mask = 0x01;
// 		for(;;) {
// 			int corr0 = 0, corr1 = 0, corrEOF = 0;
// 			for(j = 0; j < arraylen(Logic0); j += skip) {
// 				corr0 += Logic0[j]*dest[i+(j/skip)];
// 			}
// 			for(j = 0; j < arraylen(Logic1); j += skip) {
// 				corr1 += Logic1[j]*dest[i+(j/skip)];
// 			}
// 			for(j = 0; j < arraylen(FrameEOF); j += skip) {
// 				corrEOF += FrameEOF[j]*dest[i+(j/skip)];
// 			}
// 			// Even things out by the length of the target waveform.
// 			corr0 *= 4;
// 			corr1 *= 4;
	
// 			if(corrEOF > corr1 && corrEOF > corr0) {
// 	//			DbpString("EOF at %d", i);
// 				break;
// 			} else if(corr1 > corr0) {
// 				i += arraylen(Logic1)/skip;
// 				outBuf[k] |= mask;
// 			} else {
// 				i += arraylen(Logic0)/skip;
// 			}
// 			mask <<= 1;
// 			if(mask == 0) {
// 				k++;
// 				mask = 0x01;
// 			}
// 			if((i+(int)arraylen(FrameEOF)) >= 2000) {
// 				DbpString("ran off end!");
// 				break;
// 			}
// 		}
// 		if(mask != 0x01) {
// 			DbpString("sniff: error, uneven octet! (discard extra bits!)");
// 	///		DbpString("   mask=%02x", mask);
// 		}
// 	//	uint8_t str1 [8];
// 	//	itoa(k,str1);
// 	//	strncat(str1," octets read",8);
	
// 	//	DbpString(  str1);    // DbpString("%d octets", k);
	
// 	//	for(i = 0; i < k; i+=3) {
// 	//		//DbpString("# %2d: %02x ", i, outBuf[i]);
// 	//		DbpIntegers(outBuf[i],outBuf[i+1],outBuf[i+2]);
// 	//	}
	
// 		for(i = 0; i < k; i++) {
// 			receivedResponse[i] = outBuf[i];
// 		}
// 	} // "end if correlation > 0" 	(max/(arraylen(FrameSOF)/skip))
// 	return k; // return the number of bytes demodulated

// ///	DbpString("CRC=%04x", Iso15693Crc(outBuf, k-2));
// }

//-----------------------------------------------------------------------------
// Wait for commands from reader
// Stop when button is pressed
// Or return TRUE when command is captured
//-----------------------------------------------------------------------------
static int GetIso15693CommandFromReader(uint8_t *received, int *len, int maxLen)
{
    // Set FPGA mode to "simulated ISO 14443 tag", no modulation (listen
    // only, since we are receiving, not transmitting).
    // Signal field is off with the appropriate LED
    LED_D_OFF();
    FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_ISO14443A_TAGSIM_LISTEN);

    // Now run a `software UART' on the stream of incoming samples.
    Uart.output = received;
    Uart.byteCntMax = maxLen;
    Uart.state = STATE_UNSYNCD;

    for(;;) {
        WDT_HIT();

        if(BUTTON_PRESS()) return FALSE;

        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
            AT91C_BASE_SSC->SSC_THR = 0x00;
        }
        if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
            uint8_t b = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
			/*if(OutOfNDecoding((b & 0xf0) >> 4)) {
				*len = Uart.byteCnt;
				return TRUE;
			}*/
			if(OutOfNDecoding(b & 0x0f)) {
				*len = Uart.byteCnt;
				return TRUE;
			}
        }
    }
}


static void BuildIdentifyRequest(void);
//-----------------------------------------------------------------------------
// Start to read an ISO 15693 tag. We send an identify request, then wait
// for the response. The response is not demodulated, just left in the buffer
// so that it can be downloaded to a PC and processed there.
//-----------------------------------------------------------------------------
void AcquireRawAdcSamplesIso15693(void)
{
	int c = 0;
	uint8_t *dest = (uint8_t *)BigBuf;
	int getNext = 0;

	int8_t prev = 0;

	BuildIdentifyRequest();

	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);

	// Give the tags time to energize
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);
	SpinDelay(100);

	// Now send the command
	FpgaSetupSsc();
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_TX);

	c = 0;
	for(;;) {
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
			AT91C_BASE_SSC->SSC_THR = ToSend[c];
			c++;
			if(c == ToSendMax+3) {
				break;
			}
		}
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
			volatile uint32_t r = AT91C_BASE_SSC->SSC_RHR;
			(void)r;
		}
		WDT_HIT();
	}

	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);

	c = 0;
	getNext = FALSE;
	for(;;) {
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
			AT91C_BASE_SSC->SSC_THR = 0x43;
		}
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
			int8_t b;
			b = (int8_t)AT91C_BASE_SSC->SSC_RHR;

			// The samples are correlations against I and Q versions of the
			// tone that the tag AM-modulates, so every other sample is I,
			// every other is Q. We just want power, so abs(I) + abs(Q) is
			// close to what we want.
			if(getNext) {
				int8_t r;

				if(b < 0) {
					r = -b;
				} else {
					r = b;
				}
				if(prev < 0) {
					r -= prev;
				} else {
					r += prev;
				}

				dest[c++] = (uint8_t)r;

				if(c >= 2000) {
					break;
				}
			} else {
				prev = b;
			}

			getNext = !getNext;
		}
	}
}


void RecordRawAdcSamplesIso15693(void)
{
	int c = 0;
	uint8_t *dest = (uint8_t *)BigBuf;
	int getNext = 0;

	int8_t prev = 0;

	// Setup SSC
	FpgaSetupSsc();

	// Start from off (no field generated)
    	FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
    	SpinDelay(200);

	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);

	SpinDelay(100);

	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);

	c = 0;
	getNext = FALSE;
	for(;;) {
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
			AT91C_BASE_SSC->SSC_THR = 0x43;
		}
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
			int8_t b;
			b = (int8_t)AT91C_BASE_SSC->SSC_RHR;

			// The samples are correlations against I and Q versions of the
			// tone that the tag AM-modulates, so every other sample is I,
			// every other is Q. We just want power, so abs(I) + abs(Q) is
			// close to what we want.
			if(getNext) {
				int8_t r;

				if(b < 0) {
					r = -b;
				} else {
					r = b;
				}
				if(prev < 0) {
					r -= prev;
				} else {
					r += prev;
				}

				dest[c++] = (uint8_t)r;

				if(c >= 7000) {
					break;
				}
			} else {
				prev = b;
			}

			getNext = !getNext;
			WDT_HIT();
		}
	}
	Dbprintf("fin record");
}


// Initialize the proxmark as iso15k reader 
// (this might produces glitches that confuse some tags
void Iso15693InitReader() {
	LED_A_ON();
	LED_B_ON();
	LED_C_OFF();
	LED_D_OFF();
	
	// Setup SSC
	// FpgaSetupSsc();

	// Start from off (no field generated)
	FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
	SpinDelay(10);

	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);
	FpgaSetupSsc();

	// Give the tags time to energize
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);
	SpinDelay(250);

	LED_A_ON();
	LED_B_OFF();
	LED_C_OFF();
	LED_D_OFF();
}

///////////////////////////////////////////////////////////////////////
// ISO 15693 Part 3 - Air Interface
// This section basicly contains transmission and receiving of bits
///////////////////////////////////////////////////////////////////////

// Encode (into the ToSend buffers) an identify request, which is the first
// thing that you must send to a tag to get a response.
static void BuildIdentifyRequest(void)
{
	uint8_t cmd[5];

	uint16_t crc;
	// one sub-carrier, inventory, 1 slot, fast rate
	// AFI is at bit 5 (1<<4) when doing an INVENTORY
	cmd[0] = (1 << 2) | (1 << 5) | (1 << 1);
	// inventory command code
	cmd[1] = 0x01;
	// no mask
	cmd[2] = 0x00;
	//Now the CRC
	crc = Crc(cmd, 3);
	cmd[3] = crc & 0xff;
	cmd[4] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}

// uid is in transmission order (which is reverse of display order)
static void BuildReadBlockRequest(uint8_t *uid, uint8_t blockNumber )
{
	uint8_t cmd[13];

	uint16_t crc;
	// If we set the Option_Flag in this request, the VICC will respond with the secuirty status of the block
	// followed by teh block data
	// one sub-carrier, inventory, 1 slot, fast rate
	cmd[0] = (1 << 6)| (1 << 5) | (1 << 1); // no SELECT bit, ADDR bit, OPTION bit
	// READ BLOCK command code
	cmd[1] = 0x20;
	// UID may be optionally specified here
	// 64-bit UID
	cmd[2] = uid[0];
	cmd[3] = uid[1];
	cmd[4] = uid[2];
	cmd[5] = uid[3];
	cmd[6] = uid[4];
	cmd[7] = uid[5];
	cmd[8] = uid[6];
	cmd[9] = uid[7]; // 0xe0; // always e0 (not exactly unique)
	// Block number to read
	cmd[10] = blockNumber;//0x00;
	//Now the CRC
	crc = Crc(cmd, 11); // the crc needs to be calculated over 12 bytes
	cmd[11] = crc & 0xff;
	cmd[12] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}

// Universal Method for sending to and recv bytes from a tag
// 	init ... should we initialize the reader?
// 	speed ... 0 low speed, 1 hi speed 
// 	**recv will return you a pointer to the received data
// 	If you do not need the answer use NULL for *recv[] 
//	return: lenght of received data
int SendDataTag(uint8_t *send, int sendlen, int init, int speed, uint8_t **recv) {

	int samples = 0;
	int tsamples = 0;
	int wait = 0;
	int elapsed = 0;
	
	LED_A_ON();
	LED_B_ON();
	LED_C_OFF();
	LED_D_OFF();
	
	int answerLen=0;
	uint8_t *answer = (((uint8_t *)BigBuf) + 3660);
	if (recv!=NULL) memset(BigBuf + 3660, 0, 100);

	if (init) Iso15693InitReader();
	
	if (!speed) {
		// low speed (1 out of 256)
		CodeIso15693AsReader256(send, sendlen);
	} else {
		// high speed (1 out of 4)
		CodeIso15693AsReader(send, sendlen);
	}
	
	LED_A_ON();
	LED_B_OFF();
	
	TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);	
	// Now wait for a response
	if (recv!=NULL) {
		LED_A_OFF();
		LED_B_ON();
		answerLen = GetIso15693AnswerFromTag(answer, 100, &samples, &elapsed) ;	
		*recv=answer;
	}

	LED_A_OFF();
	LED_B_OFF();
	LED_C_OFF();
	LED_D_OFF();
	
	return answerLen;
}


// --------------------------------------------------------------------
// Debug Functions 
// --------------------------------------------------------------------

// Decodes a message from a tag and displays its metadata and content
#define DBD15STATLEN 48
void DbdecodeIso15693Answer(int len, uint8_t *d) {
	char status[DBD15STATLEN+1]={0};
	uint16_t crc;

	if (len>3) {
		if (d[0]&(1<<3)) 
			strncat(status,"ProtExt ",DBD15STATLEN);
		if (d[0]&1) { 
			// error
			strncat(status,"Error ",DBD15STATLEN);
			switch (d[1]) {
				case 0x01: 
					strncat(status,"01:notSupp",DBD15STATLEN);
					break;
				case 0x02: 
					strncat(status,"02:notRecog",DBD15STATLEN);
					break;
				case 0x03: 
					strncat(status,"03:optNotSupp",DBD15STATLEN);
					break;
				case 0x0f: 
					strncat(status,"0f:noInfo",DBD15STATLEN);
					break;
				case 0x10: 
					strncat(status,"10:dontExist",DBD15STATLEN);
					break;
				case 0x11: 
					strncat(status,"11:lockAgain",DBD15STATLEN);
					break;
				case 0x12: 
					strncat(status,"12:locked",DBD15STATLEN);
					break;
				case 0x13: 
					strncat(status,"13:progErr",DBD15STATLEN);
					break;
				case 0x14: 
					strncat(status,"14:lockErr",DBD15STATLEN);
					break;
				default:
					strncat(status,"unknownErr",DBD15STATLEN);
			}
			strncat(status," ",DBD15STATLEN);
		} else {
			strncat(status,"NoErr ",DBD15STATLEN);
		}
			
		crc=Crc(d,len-2);
		if ( (( crc & 0xff ) == d[len-2]) && (( crc >> 8 ) == d[len-1]) ) 
			strncat(status,"CrcOK",DBD15STATLEN);
		else
			strncat(status,"CrcFail!",DBD15STATLEN); 

		Dbprintf("%s",status);
	}
}



///////////////////////////////////////////////////////////////////////
// Functions called via USB/Client
///////////////////////////////////////////////////////////////////////

void SetDebugIso15693(uint32_t debug) {
	DEBUG=debug;
	Dbprintf("Iso15693 Debug is now %s",DEBUG?"on":"off");
	return;
}



//-----------------------------------------------------------------------------
// Simulate an ISO15693 reader, perform anti-collision and then attempt to read a sector
// all demodulation performed in arm rather than host. - greg
//-----------------------------------------------------------------------------
void ReaderIso15693(uint32_t parameter)
{
	LED_A_ON();
	LED_B_ON();
	LED_C_OFF();
	LED_D_OFF();

//DbpString(parameter);

	//uint8_t *answer0 = (((uint8_t *)BigBuf) + 3560); // allow 100 bytes per reponse (way too much)
	uint8_t *answer1 = (((uint8_t *)BigBuf) + 3660); //
	uint8_t *answer2 = (((uint8_t *)BigBuf) + 3760);
	uint8_t *answer3 = (((uint8_t *)BigBuf) + 3860);
	//uint8_t *TagUID= (((uint8_t *)BigBuf) + 3960);		// where we hold the uid for hi15reader
//	int answerLen0 = 0;
	int answerLen1 = 0;
	int answerLen2 = 0;
	int answerLen3 = 0;
	int i=0; // counter

	// Blank arrays
	memset(BigBuf + 3660, 0, 300);

	// Setup SSC
	FpgaSetupSsc();

	// Start from off (no field generated)
    	FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
    	SpinDelay(200);

	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);
	FpgaSetupSsc();

	// Give the tags time to energize
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);
	SpinDelay(200);

	LED_A_ON();
	LED_B_OFF();
	LED_C_OFF();
	LED_D_OFF();

	int samples = 0;
	int tsamples = 0;
	int wait = 0;
	int elapsed = 0;

	// FIRST WE RUN AN INVENTORY TO GET THE TAG UID
	// THIS MEANS WE CAN PRE-BUILD REQUESTS TO SAVE CPU TIME
	 uint8_t TagUID[8] = {0, 0, 0, 0, 0, 0, 0, 0};		// where we hold the uid for hi15reader

//	BuildIdentifyRequest();
//	//TransmitTo15693Tag(ToSend,ToSendMax+3,&tsamples, &wait);
//	TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);	// No longer ToSendMax+3
//	// Now wait for a response
//	responseLen0 = GetIso15693AnswerFromTag(receivedAnswer0, 100, &samples, &elapsed) ;
//	if (responseLen0 >=12) // we should do a better check than this
//	{
//		// really we should check it is a valid mesg
//		// but for now just grab what we think is the uid
//		TagUID[0] = receivedAnswer0[2];
//		TagUID[1] = receivedAnswer0[3];
//		TagUID[2] = receivedAnswer0[4];
//		TagUID[3] = receivedAnswer0[5];
//		TagUID[4] = receivedAnswer0[6];
//		TagUID[5] = receivedAnswer0[7];
//		TagUID[6] = receivedAnswer0[8]; // IC Manufacturer code
//	DbpIntegers(TagUID[6],TagUID[5],TagUID[4]);
//}

	// Now send the IDENTIFY command
	BuildIdentifyRequest();
	//TransmitTo15693Tag(ToSend,ToSendMax+3,&tsamples, &wait);
	TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);	// No longer ToSendMax+3
	// Now wait for a response
	answerLen1 = GetIso15693AnswerFromTag(answer1, 100, &samples, &elapsed) ;

	if (answerLen1 >=12) // we should do a better check than this
	{

		TagUID[0] = answer1[2];
		TagUID[1] = answer1[3];
		TagUID[2] = answer1[4];
		TagUID[3] = answer1[5];
		TagUID[4] = answer1[6];
		TagUID[5] = answer1[7];
		TagUID[6] = answer1[8]; // IC Manufacturer code
		TagUID[7] = answer1[9]; // always E0

		// Now send the SELECT command
		// since the SELECT command is optional, we should not rely on it.
////				BuildSelectRequest(TagUID);
//		TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);	// No longer ToSendMax+3
		// Now wait for a response
///		answerLen2 = GetIso15693AnswerFromTag(answer2, 100, &samples, &elapsed);

		// Now send the MULTI READ command
//		BuildArbitraryRequest(*TagUID,parameter);
///		BuildArbitraryCustomRequest(TagUID,parameter);
//		BuildReadBlockRequest(*TagUID,parameter);
//		BuildSysInfoRequest(*TagUID);
		//TransmitTo15693Tag(ToSend,ToSendMax+3,&tsamples, &wait);
///		TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);	// No longer ToSendMax+3
		// Now wait for a response
///		answerLen3 = GetIso15693AnswerFromTag(answer3, 100, &samples, &elapsed) ;

	}

	Dbprintf("%d octets read from IDENTIFY request:", answerLen1);
	DbdecodeIso15693Answer(answerLen1,answer1);
	Dbhexdump(answerLen1,answer1,true);

	// UID is reverse
	if (answerLen1>=12) 
		//Dbprintf("UID = %*D",8,TagUID," ");
		Dbprintf("UID = %02hX%02hX%02hX%02hX%02hX%02hX%02hX%02hX",TagUID[7],TagUID[6],TagUID[5],
			TagUID[4],TagUID[3],TagUID[2],TagUID[1],TagUID[0]);


	Dbprintf("%d octets read from SELECT request:", answerLen2);
	DbdecodeIso15693Answer(answerLen2,answer2);
	Dbhexdump(answerLen2,answer2,true);

	Dbprintf("%d octets read from XXX request:", answerLen3);
	DbdecodeIso15693Answer(answerLen3,answer3);
	Dbhexdump(answerLen3,answer3,true);

 
	// read all pages
	if (answerLen1>=12 && DEBUG) {
		i=0;			
		while (i<32) {  // sanity check, assume max 32 pages
			BuildReadBlockRequest(TagUID,i);
	      TransmitTo15693Tag(ToSend,ToSendMax,&tsamples, &wait);  
         answerLen2 = GetIso15693AnswerFromTag(answer2, 100, &samples, &elapsed);
			if (answerLen2>0) {
				Dbprintf("READ SINGLE BLOCK %d returned %d octets:",i,answerLen2);
				DbdecodeIso15693Answer(answerLen2,answer2);
				Dbhexdump(answerLen2,answer2,true);
				if ( *((uint32_t*) answer2) == 0x07160101 ) break; // exit on NoPageErr 
			} 
			i++;
		} 
	}

//	str2[0]=0;
//	for(i = 0; i < responseLen3; i++) {
//		itoa(str1,receivedAnswer3[i]);
//		strncat(str2,str1,8);
//	}
//	DbpString(str2);

	LED_A_OFF();
	LED_B_OFF();
	LED_C_OFF();
	LED_D_OFF();
}

// // Simulate an ISO15693 TAG, perform anti-collision and then print any reader commands
// // all demodulation performed in arm rather than host. - greg
// void SimTagIso15693(uint32_t afi, uint32_t dsfid, uint32_t eas, uint8_t *uid)
// {
// 	LED_A_ON();
// 	LED_B_ON();
// 	LED_C_OFF();
// 	LED_D_OFF();

// 	uint8_t *answer1 = (((uint8_t *)BigBuf) + 3660); //
// 	int answerLen1 = 0;

// 	// Blank arrays
// 	memset(answer1, 0, 100);

// 	// Setup SSC
// 	FpgaSetupSsc();

// 	// Start from off (no field generated)
//     	FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
//     	SpinDelay(200);

// 	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);
// 	FpgaSetupSsc();

// 	// Give the tags time to energize
// //	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER_RX_XCORR);	// NO GOOD FOR SIM TAG!!!!
// 	SpinDelay(200);

// 	LED_A_OFF();
// 	LED_B_OFF();
// 	LED_C_ON();
// 	LED_D_OFF();

// 	int samples = 0;
// 	int tsamples = 0;
// 	int wait = 0;
// 	int elapsed = 0;

// 	answerLen1 = GetIso15693AnswerFromSniff(answer1, 100, &samples, &elapsed) ;

// 	if (answerLen1 >=1) // we should do a better check than this
// 	{
		
// 	}

// 	Dbprintf("%d octets read from reader command: %x %x %x %x %x %x %x %x %x", answerLen1,
// 		answer1[0], answer1[1], answer1[2],
// 		answer1[3], answer1[4], answer1[5],
// 		answer1[6], answer1[7], answer1[8]);

// 	LED_A_OFF();
// 	LED_B_OFF();
// 	LED_C_OFF();
// 	LED_D_OFF();
// }
static int SendIClassAnswer(uint8_t *resp, int respLen, int delay)
{
	int i = 0, u = 0, d = 0;
	uint8_t b = 0;
	// return 0;
	// Modulate Manchester
	// FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_ISO14443A_TAGSIM_MOD423);
	FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_ISO14443A_TAGSIM_MOD);
	AT91C_BASE_SSC->SSC_THR = 0x00;
	FpgaSetupSsc();
	
	// send cycle
	for(;;) {
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_RXRDY)) {
			volatile uint8_t b = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
			(void)b;
		}
		if(AT91C_BASE_SSC->SSC_SR & (AT91C_SSC_TXRDY)) {
			if(d < delay) {
				b = 0x00;
				d++;
			}
			else if(i >= respLen) {
				b = 0x00;
				u++;
			} else {
				b = resp[i];
				u++;
				if(u > 1) { i++; u = 0; }
			}
			AT91C_BASE_SSC->SSC_THR = b;

			if(u > 4) break;
		}
		if(BUTTON_PRESS()) {
			break;
		}
	}

	return 0;
}

//-----------------------------------------------------------------------------
// Prepare tag messages
//-----------------------------------------------------------------------------
static void CodeIClassTagAnswer(const uint8_t *cmd, int len)
{
	int i;

	ToSendReset();

	// Send SOF
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0xff;

	for(i = 0; i < len; i++) {
		int j;
		uint8_t b = cmd[i];

		// Data bits
		for(j = 0; j < 8; j++) {
			if(b & 1) {
				ToSend[++ToSendMax] = 0x00;
				ToSend[++ToSendMax] = 0xff;
			} else {
				ToSend[++ToSendMax] = 0xff;
				ToSend[++ToSendMax] = 0x00;
			}
			b >>= 1;
		}
	}

	// Send EOF
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0xff;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0x00;
	ToSend[++ToSendMax] = 0x00;

	// Convert from last byte pos to length
	ToSendMax++;
}

void SimTagIso15693(uint32_t afi, uint32_t dsfid, uint32_t eas, uint8_t *uid)
{
	// Enable and clear the trace
	tracing = TRUE;
	traceLen = 0;
	memset(trace, 0x44, TRACE_SIZE);

	// Responses
  	uint8_t cmdRespInventory[] = { /*Flags*/0x00, /*DSFID*/dsfid, /*UID*/0x12, 0x34, 0x56, 0x78, 0x00 ,0x01, 0x04, 0xe0,/*CRC*/ 0x00, 0x00 };
	memcpy(&cmdRespInventory[2], uid, 8);
	AddCrc(cmdRespInventory, sizeof(cmdRespInventory) -2 );

	uint8_t cmdRespGetSystemInfo[] = {
	0x00, //Flags
	0x0F, /*info Flags	b1 DSFID present
						b2 AFI present
						b3 VICC memory size present
						b4 IC reference present
						b5-8 RFU
						*/ 
	0x12, 0x34, 0x56, 0x78, 0x00 ,0x01, 0x04, 0xe0, /*UID*/

	dsfid, /*DSFID*/
	afi, /*AFI*/
	0x1b, //Number of blocks is on 8 bits, allowing to specify up to 256 blocks. It is one less than the actual number of blocks. 
	0x03, //Block size is expressed in number of bytes on 5 bits, allowing to specify up to 32 bytes i.e. 256 bits. It is one less than the actual number of bytes.
	uid[1], //use uid[1] as IC reference
	0x00, 0x00 /*CRC*/};
	memcpy(&cmdRespGetSystemInfo[2], uid, 8);
	AddCrc(cmdRespGetSystemInfo, sizeof(cmdRespGetSystemInfo) -2);
	
	uint8_t cmdRespReadSinglBlock[] = { /*Flags*/0x00,/*Data*/0xDE,0xAD,0xBE,0xEF, /*CRC*/ 0x00, 0x00  };
	AddCrc(cmdRespReadSinglBlock, sizeof(cmdRespReadSinglBlock) -2);
	

	uint8_t cmdRespReadSinglBlockWithSecStatus[] = { /*Flags*/0x00, /*Block security status*/0x00, /*Data*/0xCA, 0xFE, 0xBA, 0xBE, /*CRC*/ 0x00, 0x00  };
	AddCrc(cmdRespReadSinglBlockWithSecStatus, sizeof(cmdRespReadSinglBlockWithSecStatus) -2);

	uint8_t cmdRespEASalarm[] = { /*Flags*/0x00,};
	AddCrc(cmdRespEASalarm, sizeof(cmdRespEASalarm) -2);

	uint8_t cmdRespError[] = { /*Flags*/0x00, /*error code*/0x0F, /*CRC*/ 0x00, 0x00  };
	AddCrc(cmdRespError, sizeof(cmdRespError) -2);

	uint8_t cmdResp6[] = { /*Flags*/0x00, /*CRC*/ 0x00, 0x00  };
	AddCrc(cmdResp6, sizeof(cmdResp6) -2);

	uint8_t cmdResp7[] = { /*Flags*/0x00, /*CRC*/ 0x00, 0x00  };
	AddCrc(cmdResp7, sizeof(cmdResp7) -2);


	uint8_t *resp;
	int respLen;
	uint8_t *respdata = NULL;
	int respsize = 0;

	// Prepare card messages
	ToSendMax = 0;

	// Inventory
	// ???: Takes 16 bytes for SOF/EOF and ? *16 = ??? bytes (2 bytes/bit)
	uint8_t *respInventory = (((uint8_t *)BigBuf) + FREE_BUFFER_OFFSET);
	int respInventoryLen;
	// Build a suitable reponse to the reader INVENTORY cocmmand
	CodeIClassTagAnswer(cmdRespInventory, sizeof(cmdRespInventory));
	memcpy(respInventory, ToSend, ToSendMax); respInventoryLen = ToSendMax;

	// Respond SOF -- takes 8 bytes
	uint8_t *respGetSystemInfo = (respInventory + ToSendMax +5 );
	int respGetSystemInfoLen;
	// Get System Info
	CodeIClassTagAnswer(cmdRespGetSystemInfo, sizeof(cmdRespGetSystemInfo));
	memcpy(respGetSystemInfo, ToSend, ToSendMax); respGetSystemInfoLen = ToSendMax;

// ReadSingleBlock
	// 176: Takes 16 bytes for SOF/EOF and 10 * 16 = 160 bytes (2 bytes/bit)
	uint8_t *respReadSingleBlock = (respGetSystemInfo + ToSendMax +5 );
	int respReadSingleBlockLen;
	CodeIClassTagAnswer(cmdRespReadSinglBlock, sizeof(cmdRespReadSinglBlock));
	memcpy(respReadSingleBlock, ToSend, ToSendMax); respReadSingleBlockLen = ToSendMax;

	
	// 176: Takes 16 bytes for SOF/EOF and 10 * 16 = 160 bytes (2 bytes/bit)
	uint8_t *resp3 = (respReadSingleBlock + ToSendMax +5 );
	int resp3Len;
	CodeIClassTagAnswer(cmdRespReadSinglBlockWithSecStatus, sizeof(cmdRespReadSinglBlockWithSecStatus));
	memcpy(resp3, ToSend, ToSendMax); resp3Len = ToSendMax;

	
	//
	uint8_t *respEASalarm = (resp3 + ToSendMax +5 );
	int respEASalarmLen;
	CodeIClassTagAnswer(cmdRespEASalarm, sizeof(cmdRespEASalarm));
	memcpy(respEASalarm, ToSend, ToSendMax); respEASalarmLen = ToSendMax;

	//
	uint8_t *respError = (respEASalarm + ToSendMax +5 );
	int respErrorLen;
	CodeIClassTagAnswer(cmdRespError, sizeof(cmdRespError));
	memcpy(respError, ToSend, ToSendMax); respErrorLen = ToSendMax;

	//
	uint8_t *resp6 = (respError + ToSendMax +5 );
	int resp6Len;
	CodeIClassTagAnswer(cmdResp6, sizeof(cmdResp6));
	memcpy(resp6, ToSend, ToSendMax); resp6Len = ToSendMax;

	//
	uint8_t *resp7 = (resp6 + ToSendMax +5 );
	int resp7Len;
	CodeIClassTagAnswer(cmdResp7, sizeof(cmdResp7));
	memcpy(resp7, ToSend, ToSendMax); resp7Len = ToSendMax;


	// + 1720..
  uint8_t *receivedCmd = (((uint8_t *)BigBuf) + RECV_CMD_OFFSET);
	memset(receivedCmd, 0x44, RECV_CMD_SIZE);
	int len;
	
	// We need to listen to the high-frequency, peak-detected path.
	SetAdcMuxFor(GPIO_MUXSEL_HIPKD);
	FpgaSetupSsc();

	// To control where we are in the protocol
	int cmdsRecvd = 0;

	LED_A_ON();
	for(;;) {
		LED_B_OFF();
		if(!GetIso15693CommandFromReader(receivedCmd, &len, 100)) {
			DbpString("button press");
			break;
		}

		// Okay, look at the command now.
		if(len > 2 && receivedCmd[1] == 0x01) {
		// Inventory Request
	        resp = respInventory; respLen = respInventoryLen;
	        respdata = cmdRespInventory;
	        respsize = sizeof(cmdRespInventory);
	        // Dbhexdump(respsize,respdata,false);			//never use Dbhexdump in here,will case timeout!!
	        // Dbhexdump(respLen,resp,false);
		} else if(len > 2 && receivedCmd[1] == 0x2B) {
			// Get System Information Request
			resp = respGetSystemInfo; respLen = respGetSystemInfoLen;
	        respdata = cmdRespGetSystemInfo;
	        respsize = sizeof(cmdRespGetSystemInfo);
        } else if(len > 2 && receivedCmd[1] == 0x20) {
			// Read Single Block Request
			if(receivedCmd[0] & ISO15_REQ_OPTION) {
				// request block security status
				resp = resp3; respLen = resp3Len;
		        respdata = cmdRespReadSinglBlockWithSecStatus;
		        respsize = sizeof(cmdRespReadSinglBlockWithSecStatus);
			} else {
				resp = respReadSingleBlock; respLen = respReadSingleBlockLen;
		        respdata = cmdRespReadSinglBlock;
		        respsize = sizeof(cmdRespReadSinglBlock);
	        }
	    } else if(len > 2 && receivedCmd[1] == 0x00) {	
			// Get System Information Request
			resp = respEASalarm; respLen = respEASalarmLen;
	        respdata = cmdRespEASalarm;
	        respsize = sizeof(cmdRespEASalarm);
	    } else if(len > 2 && receivedCmd[1] == 0x00) {	
			resp = respError; respLen = respErrorLen;
	        respdata = cmdRespError;
	        respsize = sizeof(cmdRespError);
	    } else if(len > 2 && receivedCmd[1] == 0x00) {	
			resp = resp6; respLen = resp6Len;
	        respdata = cmdResp6;
	        respsize = sizeof(cmdResp6);
	    } else if(len > 2 && receivedCmd[1] == 0x00) {	
			resp = resp7; respLen = resp7Len;
	        respdata = cmdResp7;
	        respsize = sizeof(cmdResp7);
		} else {
			// Never seen this command before
			Dbprintf("Unknown command received from reader (len=%d): %x %x %x %x %x %x %x %x %x",
			len,
			receivedCmd[0], receivedCmd[1], receivedCmd[2],
			receivedCmd[3], receivedCmd[4], receivedCmd[5],
			receivedCmd[6], receivedCmd[7], receivedCmd[8]);
			// Do not respond
			resp = respError; respLen = respErrorLen; //order = 0;
			respdata = cmdRespError;
			respsize = sizeof(cmdRespError);
		}

		if(cmdsRecvd > 999) {
			DbpString("1000 commands later...");
			break;
		}
		else {
			cmdsRecvd++;
		}

		if(respLen > 0) {
			SendIClassAnswer(resp, respLen, 0);
		}

		// Dbprintf("received from reader (len=%d): %x %x %x %x %x %x %x %x %x",
		// len,
		// receivedCmd[0], receivedCmd[1], receivedCmd[2],
		// receivedCmd[3], receivedCmd[4], receivedCmd[5],
		// receivedCmd[6], receivedCmd[7], receivedCmd[8]);
		Dbprintf("received from reader (len=%d):");
		Dbhexdump(len,receivedCmd,false);

		if (tracing) {
			LogTrace(receivedCmd,len, rsamples, Uart.parityBits, TRUE);
			if (respdata != NULL) {
				LogTrace(respdata,respsize, rsamples, SwapBits(GetParity(respdata,respsize),respsize), FALSE);
			}
			if(traceLen > TRACE_SIZE) {
				DbpString("Trace full");
				break;
			}
		}

		memset(receivedCmd, 0x44, RECV_CMD_SIZE);
	}

	Dbprintf("%x", cmdsRecvd);
	LED_A_OFF();
	LED_B_OFF();
}

// Since there is no standardized way of reading the AFI out of a tag, we will brute force it
// (some manufactures offer a way to read the AFI, though)
void BruteforceIso15693Afi(uint32_t speed) 
{	
	uint8_t data[20];
	uint8_t *recv=data;
	int datalen=0, recvlen=0;
		
	Iso15693InitReader();
	
	// first without AFI
	// Tags should respond wihtout AFI and with AFI=0 even when AFI is active
	
	data[0]=ISO15_REQ_SUBCARRIER_SINGLE | ISO15_REQ_DATARATE_HIGH | 
	        ISO15_REQ_INVENTORY | ISO15_REQINV_SLOT1;
	data[1]=ISO15_CMD_INVENTORY;
	data[2]=0; // mask length
	datalen=AddCrc(data,3);
	recvlen=SendDataTag(data,datalen,0,speed,&recv);
	WDT_HIT();
	if (recvlen>=12) {
		Dbprintf("NoAFI UID=%s",sprintUID(NULL,&recv[2]));
	}
	
	// now with AFI
	
	data[0]=ISO15_REQ_SUBCARRIER_SINGLE | ISO15_REQ_DATARATE_HIGH | 
	        ISO15_REQ_INVENTORY | ISO15_REQINV_AFI | ISO15_REQINV_SLOT1;
	data[1]=ISO15_CMD_INVENTORY;
	data[2]=0; // AFI
	data[3]=0; // mask length
	
	for (int i=0;i<256;i++) {
		data[2]=i & 0xFF;
		datalen=AddCrc(data,4);
		recvlen=SendDataTag(data,datalen,0,speed,&recv);
		WDT_HIT();
		if (recvlen>=12) {
			Dbprintf("AFI=%i UID=%s",i,sprintUID(NULL,&recv[2]));
		}
	}	
	Dbprintf("AFI Bruteforcing done.");
	
}

// Allows to directly send commands to the tag via the client
void DirectTag15693Command(uint32_t datalen,uint32_t speed, uint32_t recv, uint8_t data[]) {

	int recvlen=0;
	uint8_t *recvbuf=(uint8_t *)BigBuf;
//	UsbCommand n;
	
	if (DEBUG) {
		Dbprintf("SEND");
		Dbhexdump(datalen,data,true);
	}
	
	recvlen=SendDataTag(data,datalen,1,speed,(recv?&recvbuf:NULL));

	if (recv) { 
//		n.cmd=/* CMD_ISO_15693_COMMAND_DONE */ CMD_ACK;
//		n.arg[0]=recvlen>48?48:recvlen;
//		memcpy(n.d.asBytes, recvbuf, 48);
		LED_B_ON();
    cmd_send(CMD_ACK,recvlen>48?48:recvlen,0,0,recvbuf,48);
//		UsbSendPacket((uint8_t *)&n, sizeof(n));
		LED_B_OFF();	
		
		if (DEBUG) {
			Dbprintf("RECV");
			DbdecodeIso15693Answer(recvlen,recvbuf); 
			Dbhexdump(recvlen,recvbuf,true);
		}
	}

}




// --------------------------------------------------------------------
// -- Misc & deprecated functions
// --------------------------------------------------------------------

/*

// do not use; has a fix UID
static void __attribute__((unused)) BuildSysInfoRequest(uint8_t *uid)
{
	uint8_t cmd[12];

	uint16_t crc;
	// If we set the Option_Flag in this request, the VICC will respond with the secuirty status of the block
	// followed by teh block data
	// one sub-carrier, inventory, 1 slot, fast rate
	cmd[0] =  (1 << 5) | (1 << 1); // no SELECT bit
	// System Information command code
	cmd[1] = 0x2B;
	// UID may be optionally specified here
	// 64-bit UID
	cmd[2] = 0x32;
	cmd[3]= 0x4b;
	cmd[4] = 0x03;
	cmd[5] = 0x01;
	cmd[6] = 0x00;
	cmd[7] = 0x10;
	cmd[8] = 0x05;
	cmd[9]= 0xe0; // always e0 (not exactly unique)
	//Now the CRC
	crc = Crc(cmd, 10); // the crc needs to be calculated over 2 bytes
	cmd[10] = crc & 0xff;
	cmd[11] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}


// do not use; has a fix UID
static void __attribute__((unused)) BuildReadMultiBlockRequest(uint8_t *uid)
{
	uint8_t cmd[14];

	uint16_t crc;
	// If we set the Option_Flag in this request, the VICC will respond with the secuirty status of the block
	// followed by teh block data
	// one sub-carrier, inventory, 1 slot, fast rate
	cmd[0] =  (1 << 5) | (1 << 1); // no SELECT bit
	// READ Multi BLOCK command code
	cmd[1] = 0x23;
	// UID may be optionally specified here
	// 64-bit UID
	cmd[2] = 0x32;
	cmd[3]= 0x4b;
	cmd[4] = 0x03;
	cmd[5] = 0x01;
	cmd[6] = 0x00;
	cmd[7] = 0x10;
	cmd[8] = 0x05;
	cmd[9]= 0xe0; // always e0 (not exactly unique)
	// First Block number to read
	cmd[10] = 0x00;
	// Number of Blocks to read
	cmd[11] = 0x2f; // read quite a few
	//Now the CRC
	crc = Crc(cmd, 12); // the crc needs to be calculated over 2 bytes
	cmd[12] = crc & 0xff;
	cmd[13] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}

// do not use; has a fix UID
static void __attribute__((unused)) BuildArbitraryRequest(uint8_t *uid,uint8_t CmdCode)
{
	uint8_t cmd[14];

	uint16_t crc;
	// If we set the Option_Flag in this request, the VICC will respond with the secuirty status of the block1
	// followed by teh block data
	// one sub-carrier, inventory, 1 slot, fast rate
	cmd[0] =   (1 << 5) | (1 << 1); // no SELECT bit
	// READ BLOCK command code
	cmd[1] = CmdCode;
	// UID may be optionally specified here
	// 64-bit UID
	cmd[2] = 0x32;
	cmd[3]= 0x4b;
	cmd[4] = 0x03;
	cmd[5] = 0x01;
	cmd[6] = 0x00;
	cmd[7] = 0x10;
	cmd[8] = 0x05;
	cmd[9]= 0xe0; // always e0 (not exactly unique)
	// Parameter
	cmd[10] = 0x00;
	cmd[11] = 0x0a;

//	cmd[12] = 0x00;
//	cmd[13] = 0x00;	//Now the CRC
	crc = Crc(cmd, 12); // the crc needs to be calculated over 2 bytes
	cmd[12] = crc & 0xff;
	cmd[13] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}

// do not use; has a fix UID
static void __attribute__((unused)) BuildArbitraryCustomRequest(uint8_t uid[], uint8_t CmdCode)
{
	uint8_t cmd[14];

	uint16_t crc;
	// If we set the Option_Flag in this request, the VICC will respond with the secuirty status of the block
	// followed by teh block data
	// one sub-carrier, inventory, 1 slot, fast rate
	cmd[0] =   (1 << 5) | (1 << 1); // no SELECT bit
	// READ BLOCK command code
	cmd[1] = CmdCode;
	// UID may be optionally specified here
	// 64-bit UID
	cmd[2] = 0x32;
	cmd[3]= 0x4b;
	cmd[4] = 0x03;
	cmd[5] = 0x01;
	cmd[6] = 0x00;
	cmd[7] = 0x10;
	cmd[8] = 0x05;
	cmd[9]= 0xe0; // always e0 (not exactly unique)
	// Parameter
	cmd[10] = 0x05; // for custom codes this must be manufcturer code
	cmd[11] = 0x00;

//	cmd[12] = 0x00;
//	cmd[13] = 0x00;	//Now the CRC
	crc = Crc(cmd, 12); // the crc needs to be calculated over 2 bytes
	cmd[12] = crc & 0xff;
	cmd[13] = crc >> 8;

	CodeIso15693AsReader(cmd, sizeof(cmd));
}




*/


