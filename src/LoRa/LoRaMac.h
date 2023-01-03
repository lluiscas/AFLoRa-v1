#ifndef __LORAMAC_H
#define __LORAMAC_H

#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/linklayer/contract/IMACProtocol.h"
#include "inet/linklayer/base/MACProtocolBase.h"
#include "inet/common/FSMA.h"
#include "inet/common/queue/IPassiveQueue.h"
#include "LoRaMacControlInfo_m.h"
#include "LoRaMacFrame_m.h"
#include "LoRaApp/LoRaAppPacket_m.h"

#include "LoRaRadio.h"

namespace inet {

using namespace physicallayer;

/**
 * Based on CSMA class
 */

class LoRaMac : public MACProtocolBase
{
  protected:
    /**
     * @name Configuration parameters
     */
    //@{
    DevAddr address;
    // (LC)
    // moved to SimpleLoRaApp module
    //bool useACK = false;           // Discrimination and initialization of nodes with and without ACK
    double bitrate = NaN;
    int headerLength = -1;
    int ackLength = -1;
    simtime_t ackTimeout = -1;
    simtime_t waitDelay1Time = -1;
    simtime_t listening1Time = -1;
    simtime_t waitDelay2Time = -1;
    simtime_t listening2Time = -1;
    simtime_t acktimeoutTime = -1;
    int maxQueueSize = -1;
    int retryLimit = -1;
    int sequenceNumber = 0;
    //
    // (LC)
    bool ACKPending;
    int sequenceNumberRemote;
    int maxRetry;
    int countRetry;
    int presentSF;       // 12 .. 7
    double presentTP;    // 2 .. 14   (dBm)
    Hz presentCF;
    Hz presentBW;
    int presentCR;
    bool presentUseHeader;
    int presentFrmPLength;
    int newSF;
    std::string sfModePar;
    // (LC)
    enum SFModes {
            SFM0,     // SF as defined in LoRaWAN protocol specification
            SFM1,     // SF no change
            SFM2,     // SF in circular changing mode SF7 -> ... -> SF12 -> SF7 -> ...
            SFM3,     // SF increment and decrement as ACKs are not or do receivedSFM0
    };

    SFModes sfMode;

    //
    // (LC)  MAC additional Frame fields  (review)
    //bool FPending = false;
    //bool ACK = false;
    //bool ADR = false;

    //variables to control ADR
    bool evaluateADRinNode;
    int ADR_ACK_CNT = 0;
    int ADR_ACK_LIMIT = 64;
    int ADR_ACK_DELAY = 32;
    bool sendNextPacketWithADRACKReq = false;

    //@}

    /**
     * @name LoRa state variables
     * Various state information checked and modified according to the state machine.
     */
    //@{
    enum State {
        IDLE,
        TRANSMIT,
        WAIT_DELAY_1,
        LISTENING_1,
        RECEIVING_1,
        WAIT_DELAY_2,
        LISTENING_2,
        RECEIVING_2,
    };

    IRadio *radio = nullptr;
    IRadio::TransmissionState transmissionState = IRadio::TRANSMISSION_STATE_UNDEFINED;
    IRadio::ReceptionState receptionState = IRadio::RECEPTION_STATE_UNDEFINED;

    cFSM fsm;

    // (LC)
    bool dcTimeOff;
    bool txQueueFull;
    bool txQueueEmpty;
    bool txDeferred;
    bool retry;

    /** Remaining backoff period in seconds */
    simtime_t backoffPeriod = -1;

    /** Number of frame retransmission attempts. */
    int retryCounter = -1;

    /** Messages received from upper layer and to be transmitted later */
    cPacketQueue transmissionQueue;

    /** Passive queue module to request messages from */
    IPassiveQueue *queueModule = nullptr;
    //@}

    /** @name Timer messages */
    //@{
    /** Timeout after the transmission of a Data frame */
    cMessage *endTransmission = nullptr;

    /** Timeout after the reception of a Data frame */
    cMessage *endReception = nullptr;

    /** Timeout after the reception of a Data frame */
    cMessage *droppedPacket = nullptr;

    /** End of the Delay_1 */
    cMessage *endDelay_1 = nullptr;

    /** End of the Listening_1 */
    cMessage *endListening_1 = nullptr;

    /** End of the Delay_2 */
    cMessage *endDelay_2 = nullptr;

    /** End of the Listening_2 */
    cMessage *endListening_2 = nullptr;

    /** Radio state change self message. Currently this is optimized away and sent directly */
    cMessage *mediumStateChange = nullptr;

    // (LC)

    /** End of ack timeout: retransmission */
    cMessage *retransmission = nullptr;

    /** Final of duty cycle  */
    cMessage *endDCTimeOff = nullptr;


    //@}

    /** @name Statistics */
    //@{
    long numRetry;
    long numAckReceivedNoRetry;   // (LC)
    long numGivenUp;
    long numCollision;
    long numSent;
    long numSentConFrames;
    long numSentUnconFrames;
    long numReceived;
    long numSentBroadcast;
    long numReceivedBroadcast;
    long numAckReceived;           // (LC)
    long numAckSent;
    long numAckNotReceived;
    long numADRCommandReceived;
    long numDroppedUpPackets;

    // (LC)
    //Statistics of send and received packets
    cOutVector nodeDroppedDC;      // (LC)
    cOutVector sfSentCount;        // (LC)
    cOutVector tpSentCount;        // (LC)
    cOutVector reqSendPkConfd;     // (LC) vector: confirmed packets req to send by node app
    cOutVector reqSendPkUnConfd;   // (LC) vector: unconfirmed packets req to send by node app
    cOutVector rcvdAckPk;          // (LC) vector: received ACK packets by node

  public:
    simsignal_t LoRa_MacFrameSent;


    //@}


  public:
    /**
     * @name Construction functions
     */
    //@{
    virtual ~LoRaMac();
    //@}
    virtual DevAddr getAddress();
    // (LC)
    virtual double getPresentTP();
    virtual int getPresentSF();
    virtual Hz getPresentBW();
    virtual Hz getPresentCF();
    simtime_t calTimeOff(int sf, Hz bw, int pl, int crc, int cr );
    simtime_t getTimeOff(int fplen);

  protected:
    /**
     * @name Initialization functions
     */
    //@{
    /** @brief Initialization of the module and its variables */
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual InterfaceEntry *createInterfaceEntry() override;
    //@}

    /**
     * @name Message handing functions
     * @brief Functions called from other classes to notify about state changes and to handle messages.
     */
    //@{
    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleUpperPacket(cPacket *msg) override;
    virtual void handleLowerPacket(cPacket *msg) override;
    virtual void handleWithFsm(cMessage *msg);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details) override;

    virtual LoRaMacFrame *encapsulate(cPacket *msg);
    virtual cPacket *decapsulate(LoRaMacFrame *frame);
    //@}


    /**
     * @name Frame transmission functions
     */
    //@{
    virtual void sendDataFrame(LoRaMacFrame *frameToSend);
    virtual void sendAckFrame();
    //virtual void sendJoinFrame();
    //@}

    /**
     * @name Utility functions
     */
    //@{
    virtual void finishCurrentTransmission();
    virtual LoRaMacFrame *getCurrentTransmission();
    virtual void popTransmissionQueue();

    virtual bool isReceiving();
    virtual bool isAck(LoRaMacFrame *frame);
    virtual bool isBroadcast(LoRaMacFrame *msg);
    virtual bool isForUs(LoRaMacFrame *msg);

    // (LC)
    void setRetransmission();
    void manageFrame(LoRaMacFrame *frame);
    void increaseSFIfPossible();

    void turnOnReceiver(void);
    void turnOffReceiver(void);
    //@}
};

} // namespace inet

#endif // ifndef __LORAMAC_H
