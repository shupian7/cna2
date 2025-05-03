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

typedef struct {
  struct pkt packet;         /* The packet data */
  bool acked;                /* Whether this packet has been acknowledged */
  bool sent;                 /* Whether this packet has been sent */
  int timer_id;              /* ID of timer associated with this packet (for future use) */
} SenderPacketStatus;

static SenderPacketStatus window_A[WINDOWSIZE];  /* Window of packets at sender */
static int window_base_A;                        /* Base sequence number of sender window */
static int next_seq_A;                           /* Next sequence number to use */
static int packets_in_window;                    /* Number of packets currently in the window */
static int oldest_unacked;                       /* Position in window of oldest unacked packet */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int window_pos;
  
  /* Check if window is full */
  if (packets_in_window < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");
    
    /* Calculate position in window array */
    window_pos = (next_seq_A - window_base_A) % WINDOWSIZE;
    if (window_pos < 0) {
      window_pos += WINDOWSIZE;
    }
    
    /* Create packet */
    sendpkt.seqnum = next_seq_A;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) {
      sendpkt.payload[i] = message.data[i];
    }
    sendpkt.checksum = ComputeChecksum(sendpkt);
    
    /* Store packet in window */
    window_A[window_pos].packet = sendpkt;
    window_A[window_pos].acked = false;
    window_A[window_pos].sent = true;
    packets_in_window++;
    
    /* Update oldest unacked if this is the first packet in the window */
    if (packets_in_window == 1) {
      oldest_unacked = window_pos;
    }
    
    /* Send packet to layer 3 */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);
    
    /* Start timer if this is the first packet in the window */
    if (window_pos == oldest_unacked) {
      starttimer(A, RTT);
    }
    
    /* Increment next sequence number */
    next_seq_A = (next_seq_A + 1) % SEQSPACE;
  }
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
  int window_pos;
  int i, slide_count = 0;
  bool timer_restart = false;
  
  /* Check if packet is corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;
    
    /* Calculate position in window */
    window_pos = (packet.acknum - window_base_A) % WINDOWSIZE;
    if (window_pos < 0) {
      window_pos += WINDOWSIZE;
    }
    
    /* Check if ACK is within current window */
    if (packet.acknum >= window_base_A && 
        packet.acknum < (window_base_A + WINDOWSIZE) % SEQSPACE) {
      
      /* Mark packet as acknowledged if not already done */
      if (!window_A[window_pos].acked && window_A[window_pos].sent) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        
        window_A[window_pos].acked = true;
        new_ACKs++;
        
        /* Check if we can slide the window */
        if (packet.acknum == window_base_A) {
          /* Find how many consecutive packets have been ACKed */
          for (i = 0; i < WINDOWSIZE; i++) {
            int pos = (oldest_unacked + i) % WINDOWSIZE;
            if (window_A[pos].acked) {
              slide_count++;
            } else {
              break;
            }
          }
          
          /* Slide window */
          if (slide_count > 0) {
            /* Stop current timer */
            stoptimer(A);
            
            /* Update window base */
            window_base_A = (window_base_A + slide_count) % SEQSPACE;
            
            /* Clear acked entries and reduce packet count */
            for (i = 0; i < slide_count; i++) {
              int pos = (oldest_unacked + i) % WINDOWSIZE;
              window_A[pos].acked = false;
              window_A[pos].sent = false;
              packets_in_window--;
            }
            
            /* Find new oldest unacked packet */
            oldest_unacked = (oldest_unacked + slide_count) % WINDOWSIZE;
            
            /* Restart timer if there are still unacked packets */
            if (packets_in_window > 0) {
              starttimer(A, RTT);
            }
          }
        }
      } else {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      }
    }
  } else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0) {
    printf("----A: time out,resend packets!\n");
    printf("---A: resending packet %d\n", window_A[oldest_unacked].packet.seqnum);
  }
  
  /* Resend the oldest unacknowledged packet */
  tolayer3(A, window_A[oldest_unacked].packet);
  packets_resent++;
  
  /* Restart timer */
  starttimer(A, RTT);
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* Initialize sender variables */
  window_base_A = 0;
  next_seq_A = 0;
  packets_in_window = 0;
  oldest_unacked = 0;
  
  /* Initialize window array */
  for (i = 0; i < WINDOWSIZE; i++) {
    window_A[i].acked = false;
    window_A[i].sent = false;
    window_A[i].timer_id = -1;
    window_A[i].packet.seqnum = -1;
    window_A[i].packet.acknum = NOTINUSE;
  }
}


/********* Receiver (B) variables and procedures ************/

typedef struct {
  struct pkt packet;     /* The packet data */
  bool received;         /* Whether this packet has been received */
} ReceiverPacketStatus;

static ReceiverPacketStatus window_B[WINDOWSIZE];   /* Window of packets at receiver */
static int window_base_B;                           /* Base sequence number of receiver window */
static int delivered_count;                         /* Count of packets delivered to layer 5 */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt ack_pkt;
  int window_pos;
  int i, deliver_count;
  
  /* Create ACK packet regardless of corruption */
  ack_pkt.seqnum = NOTINUSE;
  for (i = 0; i < 20; i++) {
    ack_pkt.payload[i] = '0';
  }
  
  /* Check if packet is corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    /* Calculate position in window */
    window_pos = (packet.seqnum - window_base_B) % WINDOWSIZE;
    if (window_pos < 0) {
      window_pos += WINDOWSIZE;
    }
    
    /* Check if packet is within current window */
    if (packet.seqnum >= window_base_B && 
        packet.seqnum < (window_base_B + WINDOWSIZE) % SEQSPACE) {
      
      /* Store packet if not already received */
      if (!window_B[window_pos].received) {
        packets_received++;
        
        /* Save packet in window */
        window_B[window_pos].packet = packet;
        window_B[window_pos].received = true;
        
        /* Check if we can deliver packets in order */
        deliver_count = 0;
        for (i = 0; i < WINDOWSIZE; i++) {
          if (window_B[i].received && 
              window_B[i].packet.seqnum == (window_base_B + i) % SEQSPACE) {
            deliver_count++;
          } else {
            break;
          }
        }
        
        /* Deliver consecutive packets to layer 5 */
        for (i = 0; i < deliver_count; i++) {
          tolayer5(B, window_B[i].packet.payload);
        }
        
        /* Slide window if we delivered any packets */
        if (deliver_count > 0) {
          delivered_count += deliver_count;
          
          /* Update window base */
          window_base_B = (window_base_B + deliver_count) % SEQSPACE;
          
          /* Shift window contents */
          for (i = 0; i < WINDOWSIZE - deliver_count; i++) {
            window_B[i] = window_B[i + deliver_count];
          }
          
          /* Clear vacated slots */
          for (i = WINDOWSIZE - deliver_count; i < WINDOWSIZE; i++) {
            window_B[i].received = false;
          }
        }
      }
    }
    
    /* Always send ACK for the current packet */
    ack_pkt.acknum = packet.seqnum;
  } else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    
    /* For corrupted packets, we'll send an ACK for the last correctly received packet */
    ack_pkt.acknum = (window_base_B == 0) ? SEQSPACE - 1 : window_base_B - 1;
  }
  
  /* Compute checksum and send ACK */
  ack_pkt.checksum = ComputeChecksum(ack_pkt);
  tolayer3(B, ack_pkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  /* Initialize receiver variables */
  window_base_B = 0;
  delivered_count = 0;
  
  /* Initialize window array */
  for (i = 0; i < WINDOWSIZE; i++) {
    window_B[i].received = false;
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