#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
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
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE (2*WINDOWSIZE)      /* The serial number space of the SR is at least twice the size of the window, otherwise it is impossible to distinguish between old and new packages. */
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
static int seq_a; 
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int seqfirst = seq_a;
  int seqlast = (seq_a + WINDOWSIZE-1) % SEQSPACE;
  int index;

  /* Check if A_nextseqnum is within the sender window */
  bool in_window = false;
  if (seqfirst <= seqlast) {
    if (A_nextseqnum >= seqfirst && A_nextseqnum <= seqlast)
      in_window = true;
  } else {
    if (A_nextseqnum >= seqfirst || A_nextseqnum <= seqlast)
      in_window = true;
  }

  /* if not blocked waiting on ACK */
  if (in_window)
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
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    if (A_nextseqnum >= seqfirst)
      index = A_nextseqnum - seqfirst;
    else
      index = WINDOWSIZE - seqfirst + A_nextseqnum;
    buffer[index] = sendpkt;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (A_nextseqnum == seqfirst)
      starttimer(A,RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
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
  int i, ack_shift = 0;
  int rel_index, seq_base, seq_end;

  /* if received ACK is not corrupted */
  if (IsCorrupted(packet) == false)
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    seq_base = seq_a;

    seq_end = (seq_a + WINDOWSIZE - 1) % SEQSPACE;

    /* check if ACK is within current window */
    int in_window = ((seq_base <= seq_end && packet.acknum >= seq_base && packet.acknum <= seq_end) ||
                     (seq_base > seq_end && (packet.acknum >= seq_base || packet.acknum <= seq_end)));

    if (in_window)
    {
      /* calculate relative index in circular buffer */
      rel_index = (packet.acknum >= seq_base)
                      ? packet.acknum - seq_base
                      : WINDOWSIZE - (seq_base - packet.acknum);

      /* new ACK */
      if (buffer[rel_index].acknum == NOTINUSE)
      {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        buffer[rel_index].acknum = packet.acknum;
        windowcount--;
      }
      else
      {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      }

      /* only slide window if ACK matches the base sequence number */
      if (packet.acknum == seq_base)
      {
        /* count how many consecutive acks from start of buffer */
        for (i = 0; i < WINDOWSIZE; ++i)
        {
          if (buffer[i].acknum != NOTINUSE && strcmp(buffer[i].payload, "") != 0)
            ack_shift++;
          else
            break;
        }

        /* move base seq number */
        seq_a = (seq_a + ack_shift) % SEQSPACE;

        /* shift buffer contents */
        for (i = 0; i < WINDOWSIZE - ack_shift; ++i)
        {
          buffer[i] = buffer[i + ack_shift];
        }
        for (; i < WINDOWSIZE; ++i)
        {
          buffer[i].acknum = NOTINUSE;
          memset(buffer[i].payload, 0, sizeof(buffer[i].payload));
        }

        /* restart timer if needed */
        stoptimer(A);
        if (windowcount > 0)
          starttimer(A, RTT);
      }
    }
  }
  else
  {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
  {
    printf("----A: time out,resend packets!\n");
    printf("---A: resending packet %d\n", (buffer[0]).seqnum);
  }
  tolayer3(A, buffer[0]);
  packets_resent++;
  starttimer(A, RTT);
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  seq_a = 0;   /* windowlast is where the last packet sent is stored.
		     new packets are placed in winlast + 1
		     so initially this is set to -1
		   */
  windowcount = 0;
  
}



/********* Receiver (B)  variables and procedures ************/

static struct pkt buffer_b[WINDOWSIZE];    
static int seq_b;        
static int receivelast; 


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  int pckcount = 0;
  struct pkt sendpkt;
  int i;
  int seqfirst;
  int seqlast;
  int index;
  /* if received packet is not corrupted */
  if (IsCorrupted(packet) == false)
  {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    packets_received++;
    /*create sendpkt*/
    /* send an ACK for the received packet */
    sendpkt.acknum = packet.seqnum;
    sendpkt.seqnum = NOTINUSE;
    /* we don't have any data to send.  fill payload with 0's */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
    /* computer checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);
    /*send ack*/
    tolayer3(B, sendpkt);
    /* need to check if new packet or duplicate */
    seqfirst = seq_b;
    seqlast = (seq_b + WINDOWSIZE-1) % SEQSPACE;

    /*see if the packet received is inside the window*/
    if (((seqfirst <= seqlast) && (packet.seqnum >= seqfirst && packet.seqnum <= seqlast)) ||
        ((seqfirst > seqlast) && (packet.seqnum >= seqfirst || packet.seqnum <= seqlast)))
    {

      /*get index*/
      if (packet.seqnum >= seqfirst)
        index = packet.seqnum - seqfirst;
      else
        index = WINDOWSIZE - seqfirst + packet.seqnum;
      /*keep receivelast */
      receivelast = receivelast > index ? receivelast:index;

      /*if not duplicate, save to buffer*/

      if (strcmp(buffer_b[index].payload, packet.payload) !=0)
      {
        /*buffer it*/
        packet.acknum = packet.seqnum;
        buffer_b[index] = packet;
        /*if it is the base*/
        if (packet.seqnum == seqfirst){
          for (i = 0; i < WINDOWSIZE; i++)
          {
            if (buffer_b[i].acknum >= 0 && strcmp(buffer_b[i].payload, "")!= 0)
              pckcount++;
            else
              break;
          }
          /* update state variables */
          seq_b = (seq_b + pckcount) % SEQSPACE;
          /*update buffer*/
          for (i = 0; i <WINDOWSIZE; i++)
          {
            if ((i + pckcount) <= (receivelast+1))
              buffer_b[i] = buffer_b[i + pckcount];
          }

        }
        /* deliver to receiving application */
        tolayer5(B, packet.payload);
      }
    }
  }
}
/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  /* initialise B's window, buffer and sequence number */
  seq_b = 0;   /*record the first seq num of the window*/
  receivelast = -1;
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
