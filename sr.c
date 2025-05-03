#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added SR implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE (2*WINDOWSIZE)  /* the min sequence space for SR must be at least windowsize * 2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int baseseqnum;                 /* base sequence number (first in window) */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool timer_active[WINDOWSIZE];  /* array to track active timers for each packet */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int index;
  int seqfirst = baseseqnum;
  int seqlast = (baseseqnum + WINDOWSIZE-1) % SEQSPACE;

  /* if the A_nextseqnum is inside the window */
  if (((seqfirst <= seqlast) && (A_nextseqnum >= seqfirst && A_nextseqnum <= seqlast)) ||
      ((seqfirst > seqlast) && (A_nextseqnum >= seqfirst || A_nextseqnum <= seqlast)))
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    if (A_nextseqnum >= seqfirst)
      index = A_nextseqnum - seqfirst;
    else
      index = WINDOWSIZE - seqfirst + A_nextseqnum;
    
    buffer[index] = sendpkt;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer for this packet */
    timer_active[index] = true;
    starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ackcount = 0;
  int i, j;
  int seqfirst, seqlast;
  int index;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* need to check if new ACK or duplicate */
    seqfirst = baseseqnum;
    seqlast = (baseseqnum + WINDOWSIZE - 1) % SEQSPACE;

    /* check case when seqnum has and hasn't wrapped */
    if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
        ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast)))
    {
      /* calculate position in window buffer */
      if (packet.acknum >= seqfirst)
        index = packet.acknum - seqfirst;
      else
        index = WINDOWSIZE - seqfirst + packet.acknum;

      /* if packet has not been ACKed yet */
      if (buffer[index].acknum == NOTINUSE) {
        /* packet is a new ACK */
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        windowcount--;
        buffer[index].acknum = packet.acknum;
        
        /* stop timer for this packet */
        if (timer_active[index]) {
          stoptimer(A);
          timer_active[index] = false;
        }

        /* check if we can slide the window */
        if (packet.acknum == seqfirst) {
          /* count consecutive ACKs received starting from baseseqnum */
          for (i = 0; i < WINDOWSIZE; i++) {
            if (buffer[i].acknum != NOTINUSE)
              ackcount++;
            else
              break;
          }

          /* slide window by ackcount positions */
          baseseqnum = (baseseqnum + ackcount) % SEQSPACE;

          /* update buffer and timer status */
          for (i = 0; i < WINDOWSIZE - ackcount; i++) {
            buffer[i] = buffer[i + ackcount];
            timer_active[i] = timer_active[i + ackcount];
          }

          /* clear the rest of the buffer and timers */
          for (i = WINDOWSIZE - ackcount; i < WINDOWSIZE; i++) {
            buffer[i].acknum = NOTINUSE;
            timer_active[i] = false;
          }

          /* restart timer for the oldest unacknowledged packet if needed */
          for (i = 0; i < WINDOWSIZE; i++) {
            if (buffer[i].acknum == NOTINUSE && buffer[i].seqnum != -1) {
              starttimer(A, RTT);
              timer_active[i] = true;
              break;
            }
          }
        }
      }
    }
    else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  int seqfirst = baseseqnum;

  /* in SR, we only resend the oldest unacknowledged packet */
  for (i = 0; i < WINDOWSIZE; i++) {
    if (buffer[i].acknum == NOTINUSE && buffer[i].seqnum != -1) {
      if (TRACE > 0) {
        printf("----A: time out,resend packets!\n");
        printf("---A: resending packet %d\n", buffer[i].seqnum);
      }

      tolayer3(A, buffer[i]);
      packets_resent++;
      
      /* restart timer for this packet */
      starttimer(A, RTT);
      timer_active[i] = true;
      
      break;
    }
  }
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  baseseqnum = 0;
  windowcount = 0;
  
  /* initialize buffer and timers */
  for (i = 0; i < WINDOWSIZE; i++) {
    buffer[i].acknum = NOTINUSE;
    buffer[i].seqnum = -1;
    timer_active[i] = false;
  }
}


/********* Receiver (B) variables and procedures ************/

static struct pkt buffer_b[WINDOWSIZE];  /* array for storing packets waiting for in-order delivery */
static int baseseqnum_b;                 /* base sequence number (first in receiver window) */
static bool received[WINDOWSIZE];        /* array to track received packets */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i, j;
  int seqfirst, seqlast;
  int index;
  int count = 0;

  /* if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    /* send an ACK for the received packet */
    sendpkt.acknum = packet.seqnum;
    sendpkt.seqnum = NOTINUSE;
    
    /* we don't have any data to send. fill payload with 0's */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
      
    /* compute checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);
    
    /* send ACK */
    tolayer3(B, sendpkt);
    
    /* calculate window boundaries */
    seqfirst = baseseqnum_b;
    seqlast = (baseseqnum_b + WINDOWSIZE - 1) % SEQSPACE;
    
    /* check if packet is within the window */
    if (((seqfirst <= seqlast) && (packet.seqnum >= seqfirst && packet.seqnum <= seqlast)) ||
        ((seqfirst > seqlast) && (packet.seqnum >= seqfirst || packet.seqnum <= seqlast))) {
      
      /* calculate position in window buffer */
      if (packet.seqnum >= seqfirst)
        index = packet.seqnum - seqfirst;
      else
        index = WINDOWSIZE - seqfirst + packet.seqnum;
      
      /* if packet is not a duplicate */
      if (!received[index]) {
        packets_received++;
        
        /* store packet and mark as received */
        buffer_b[index] = packet;
        received[index] = true;
        
        /* check if we can deliver packets in-order */
        for (i = 0; i < WINDOWSIZE; i++) {
          if (received[i])
            count++;
          else
            break;
        }
        
        /* deliver all consecutive packets to upper layer */
        for (i = 0; i < count; i++) {
          /* deliver to receiving application */
          tolayer5(B, buffer_b[i].payload);
        }
        
        /* slide window */
        if (count > 0) {
          /* update base sequence number */
          baseseqnum_b = (baseseqnum_b + count) % SEQSPACE;
          
          /* shift buffer and received flags */
          for (i = 0; i < WINDOWSIZE - count; i++) {
            buffer_b[i] = buffer_b[i + count];
            received[i] = received[i + count];
          }
          
          /* clear the rest of the buffer and flags */
          for (i = WINDOWSIZE - count; i < WINDOWSIZE; i++) {
            received[i] = false;
          }
        }
      }
    }
  }
  else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  /* initialize variables */
  baseseqnum_b = 0;
  
  /* initialize buffer and received flags */
  for (i = 0; i < WINDOWSIZE; i++) {
    received[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}