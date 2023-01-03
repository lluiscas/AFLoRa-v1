//
// Copyright (C) 2016 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/common/ModuleAccess.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/UserPriority.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"

#include "LoRaMac.h"

namespace inet {

Define_Module(LoRaMac);

LoRaMac::~LoRaMac()
{
    cancelAndDelete(endTransmission);
    cancelAndDelete(endReception);
    cancelAndDelete(droppedPacket);
    cancelAndDelete(endDelay_1);
    cancelAndDelete(endListening_1);
    cancelAndDelete(endDelay_2);
    cancelAndDelete(endListening_2);
    cancelAndDelete(mediumStateChange);
    cancelAndDelete(retransmission);   // (LC)
    cancelAndDelete(endDCTimeOff);     // (LC)
}

/****************************************************************
 * Initialization functions.
 */
void LoRaMac::initialize(int stage)
{
    MACProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage 0\n";

        maxQueueSize = par("maxQueueSize");
        headerLength = par("headerLength");
        ackLength = par("ackLength");
        ackTimeout = par("ackTimeout");
        retryLimit = par("retryLimit");

        // (LC)
        waitDelay1Time = 1;
        listening1Time = 0.2;   // original value: 1;
        waitDelay2Time = 1;
        listening2Time = 0.2;   // 1;
        // (LC)
        acktimeoutTime = (intuniform(0,2))+1;   // 1 .. 3 ; (uniforme(0,2))+1
        maxRetry = 8;
        countRetry = 0;
        numADRCommandReceived = 0;
        // (LC)
        // moved to SimpleLoRaApp module
        // useAck = par("confirmedTx");
        //
        // moved from SimpleLoRaApp.cc
        presentTP = par("initialLoRaTP").doubleValue();
        presentSF = par("initialLoRaSF");
        newSF = presentSF;
        presentBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        presentCR = par("initialLoRaCR");
        presentCF = inet::units::values::Hz(par("initialLoRaCF").doubleValue());
        presentUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");

        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = DevAddr::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);
        registerInterface();

        // (LC)
        sfModePar = par("sfModePar").stdstringValue();
        if(sfModePar == "SFM0")
            sfMode = SFM0;
        else if(sfModePar == "SFM1")
            sfMode = SFM1;
        else if(sfModePar == "SFM2")
            sfMode = SFM2;
        else if(sfModePar == "SFM3")
            sfMode = SFM3;
        else
            sfMode = SFM0;

        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(LoRaRadio::droppedPacket, this);
        radio = check_and_cast<IRadio *>(radioModule);

        //LoRa_MacFrameSent = registerSignal("LoRa_MacFrameSent");

        // initialize self messages
        endTransmission = new cMessage("Transmission");
        endReception = new cMessage("Reception");
        droppedPacket = new cMessage("Dropped Packet");
        endDelay_1 = new cMessage("Delay_1");
        endListening_1 = new cMessage("Listening_1");
        endDelay_2 = new cMessage("Delay_2");
        endListening_2 = new cMessage("Listening_2");
        mediumStateChange = new cMessage("MediumStateChange");
        // (LC)
        retransmission = new cMessage("Retransmission");
        endDCTimeOff = new cMessage("EndDCTimeOff");

        // set up internal queue
        transmissionQueue.setName("transmissionQueue");
        //transmissionQueue.setMaxPacketLength(maxQueueSize);

        // state variables
        fsm.setName("LoRaMac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;

        // sequence number for messages
        sequenceNumber = 0;
        // (LC)
        sequenceNumberRemote = 0;
        ACKPending = false;
        retry = false;
        dcTimeOff = false;
        txQueueFull = false;
        txQueueEmpty = true;
        txDeferred = false;

        // statistics
        numRetry = 0;
        numAckReceivedNoRetry = 0;
        numGivenUp = 0;
        // TODO: delete numCollisions. It is counted from radio module
        numCollision = 0;
        numSent = 0;
        numSentConFrames = 0;
        numSentUnconFrames = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;
        numAckReceived = 0;
        numAckSent = 0;
        numAckNotReceived = 0;
        // (LC)
        numDroppedUpPackets = 0;
        nodeDroppedDC.setName("Node Dropped DC");
        sfSentCount.setName("SF node count");
        tpSentCount.setName("TP tx frame");
        reqSendPkConfd.setName("Req send con pk");
        reqSendPkUnConfd.setName("Req send uncon pk");
        rcvdAckPk.setName("Recvd ACK pk");

        // initialize watches
        WATCH(fsm);
        WATCH(backoffPeriod);
        WATCH(retryCounter);
        WATCH(numRetry);
        WATCH(numAckReceivedNoRetry);
        WATCH(numGivenUp);
        WATCH(numCollision);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numSentBroadcast);
        WATCH(numReceivedBroadcast);
        WATCH(numAckReceived);
        WATCH(numAckSent);
        WATCH(numAckNotReceived);
        WATCH(presentSF);
        WATCH(presentTP);
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        LoRa_MacFrameSent = registerSignal("LoRa_MacFrameSent");
    }
}

void LoRaMac::finish()
{
    recordScalar("numRetry", numRetry);
    recordScalar("numAckReceivedNoRetry", numAckReceivedNoRetry);
    recordScalar("numGivenUp", numGivenUp);
    //recordScalar("numCollision", numCollision);
    recordScalar("numSent", numSent);
    recordScalar("numSentConFrames", numSentConFrames);
    recordScalar("numSentUnconFrames", numSentUnconFrames);
    recordScalar("numReceived", numReceived);
    recordScalar("numSentBroadcast", numSentBroadcast);
    recordScalar("numReceivedBroadcast", numReceivedBroadcast);
    recordScalar("numAckReceived", numAckReceived);
    recordScalar("numAckSent", numAckSent);
    recordScalar("numAckNotReceived", numAckNotReceived);
    recordScalar("numDroppedUpPackets", numDroppedUpPackets);
    recordScalar("numPckTxPending", txQueueEmpty?0:1);
    recordScalar("numADRCommandReceived", numADRCommandReceived);
}

InterfaceEntry *LoRaMac::createInterfaceEntry()
{
    InterfaceEntry *e = new InterfaceEntry(this);

    // data rate
    e->setDatarate(bitrate);

    // capabilities
    e->setMtu(par("mtu"));
    e->setMulticast(true);
    e->setBroadcast(true);
    e->setPointToPoint(false);

    return e;
}

/****************************************************************
 * Message handling functions.
 */
void LoRaMac::handleSelfMessage(cMessage *msg)
{
    EV << "received self message: " << msg << endl;

    if ( msg == endDCTimeOff )
        dcTimeOff = false;
    if ( msg == retransmission )
        retry = true;
    if ( ACKPending )
        if ( !dcTimeOff && retry )
            txDeferred = false;
        else
            txDeferred = true;
    else
        if ( dcTimeOff )
            txDeferred = true;
        else
            txDeferred = false;
    handleWithFsm(msg);
}

void LoRaMac::handleUpperPacket(cPacket *msg)
{
    LoRaMacControlInfo *cInfo = check_and_cast<LoRaMacControlInfo *>(msg->getControlInfo());
    // (LC)
    if (simTime() >= getSimulation()->getWarmupPeriod()) {
        if (cInfo->getMType()==CONF_DATA_UP)
            reqSendPkConfd.record(1);
        if (cInfo->getMType()==UCONF_DATA_UP)
            reqSendPkUnConfd.record(1);
    }
    if ( txQueueFull )
    {
        if (simTime() >= getSimulation()->getWarmupPeriod()) {
        numDroppedUpPackets++;      //  Packets from ED App dropped for DC
        nodeDroppedDC.record(1);    //
        }
        EV << "Dropped Up frame. Transmission Queue is full \n";
        delete msg;
    }
    else {
        LoRaMacFrame *frame = encapsulate(msg);
        frame->setLoRaTP(presentTP);     // (LC) TP and SF directly controlled from MAC
        frame->setLoRaSF(presentSF);
        frame->setLoRaCF(presentCF);
        frame->setLoRaBW(presentBW);
        frame->setLoRaCR(presentCR);
        frame->setLoRaUseHeader(presentUseHeader);
        frame->setACK(false);                       // (LC)
        frame->setMType(cInfo->getMType());         // (LC)
        presentFrmPLength = cInfo->getFrmPLength();  // (LC)
        frame->setFrmPLength(presentFrmPLength);     // (LC)
        frame->setSequenceNumber(sequenceNumber);
        frame->setReceiverAddress(DevAddr::BROADCAST_ADDRESS);
        ++sequenceNumber;
        frame->setLoRaUseHeader(cInfo->getLoRaUseHeader());
        frame->setFOpsLen(NO_OPTIONS);
        frame->setADRACKReq(false);
        if ( evaluateADRinNode )
        {
            frame->setADR(true);
            if ( sendNextPacketWithADRACKReq )
                frame->setADRACKReq(true);
            sendNextPacketWithADRACKReq = false;
        }
        // (LC) TODO: consider if command LinkADRAns need to be tx at this point.
        //
        EV << "Frame " << frame << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;
        transmissionQueue.insert(frame);
        txQueueEmpty = transmissionQueue.isEmpty();
        if (transmissionQueue.getLength() == maxQueueSize )
            txQueueFull = true;
        else
            txQueueFull = false;
        handleWithFsm(frame);
    }
}

void LoRaMac::handleLowerPacket(cPacket *msg)
{
    if( (fsm.getState() == RECEIVING_1) || (fsm.getState() == RECEIVING_2))
        handleWithFsm(msg);
    else delete msg;    // (LC) TODO: review delete msg
}

void LoRaMac::handleWithFsm(cMessage *msg)
{
    LoRaMacFrame *frame = dynamic_cast<LoRaMacFrame*>(msg);
    //LoRaMacFrame *frame = check_cast<LoRaMacFrame*>(msg);
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Idle-Transmit-UpperMsg,
                                  isUpperMessage(msg) && !txDeferred,
                                  TRANSMIT,
            );
            FSMA_Event_Transition(Idle-Transmit-Pending,
                                  ((msg == endDCTimeOff || msg == retransmission) && !txDeferred && !txQueueEmpty),
                                  TRANSMIT,
            );
        }
        FSMA_State(TRANSMIT)
        {
            FSMA_Enter(sendDataFrame(getCurrentTransmission()));
            FSMA_Event_Transition(Transmit-Wait_Delay_1,
                                  msg == endTransmission,
                                  WAIT_DELAY_1,
               finishCurrentTransmission();
               numSent++;
            );
        }
        FSMA_State(WAIT_DELAY_1)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Wait_Delay_1-Listening_1,
                                  msg == endDelay_1 || endDelay_1->isScheduled() == false,
                                  LISTENING_1,
            );
        }
        FSMA_State(LISTENING_1)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_1-Wait_Delay_2,
                                  msg == endListening_1 || endListening_1->isScheduled() == false,
                                  WAIT_DELAY_2,
            );
            FSMA_Event_Transition(Listening_1-Receiving1,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_1,
            );
        }
        FSMA_State(RECEIVING_1)
        {
            FSMA_Event_Transition(Receive-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_1,
                delete frame;
            );
            FSMA_Event_Transition(Receive-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                manageFrame(frame);  // (LC)
                sendUp(decapsulate(check_and_cast<LoRaMacFrame *>(frame)));
                numReceived++;
                cancelEvent(endListening_1);
                cancelEvent(endDelay_2);
                cancelEvent(endListening_2);
            );
            FSMA_Event_Transition(Receive-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_1,
            );
        }
        FSMA_State(WAIT_DELAY_2)
        {
            FSMA_Enter(turnOffReceiver());
            FSMA_Event_Transition(Wait_Delay_2-Listening_2,
                                  msg == endDelay_2 || endDelay_2->isScheduled() == false,
                                  LISTENING_2,
            );
        }
        FSMA_State(LISTENING_2)   // TODO !!!!!!!
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_2-idle,
                                  msg == endListening_2 || endListening_2->isScheduled() == false,
                                  IDLE,
                                  setRetransmission();
            );
            FSMA_Event_Transition(Listening_2-Receiving2,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_2,
            );
        }
        FSMA_State(RECEIVING_2)
        {
            FSMA_Event_Transition(Receive2-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_2,
                delete frame;
            );
            FSMA_Event_Transition(Receive2-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                manageFrame(frame);
                sendUp(decapsulate(check_and_cast<LoRaMacFrame *>(frame)));
                numReceived++;
                cancelEvent(endListening_2);
            );
            FSMA_Event_Transition(Receive2-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_2,
            );
        }
    }
}

void LoRaMac::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::receptionStateChangedSignal) {
        IRadio::ReceptionState newRadioReceptionState = (IRadio::ReceptionState)value;
        if (receptionState == IRadio::RECEPTION_STATE_RECEIVING) {
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        }
        receptionState = newRadioReceptionState;
        handleWithFsm(mediumStateChange);
    }
    else if (signalID == LoRaRadio::droppedPacket) {
        radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        handleWithFsm(droppedPacket);
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            handleWithFsm(endTransmission);
            radio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
        }
        transmissionState = newRadioTransmissionState;
    }
}

LoRaMacFrame *LoRaMac::encapsulate(cPacket *msg)
{
    LoRaMacFrame *frame = new LoRaMacFrame(msg->getName());

    frame->setByteLength(headerLength);
    frame->setArrival(msg->getArrivalModuleId(), msg->getArrivalGateId());

    frame->setTransmitterAddress(address);

    frame->encapsulate(msg);

    return frame;
}

cPacket *LoRaMac::decapsulate(LoRaMacFrame *frame)
{
    cPacket *payload = frame->decapsulate();


    delete frame;
    return payload;
}

/****************************************************************
 * Frame sender functions.
 */
void LoRaMac::sendDataFrame(LoRaMacFrame *frameToSend)
{
    EV << "sending Data frame\n";

    if ( ACKPending )
    {
        EV << "as a retransmission\n";
        numRetry++;
    }
    else if (frameToSend->getMType() == CONF_DATA_UP)
    {
        ACKPending = true;
    }

    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    LoRaMacFrame *frameCopy = frameToSend->dup();

    LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
    ctrl->setSrc(frameCopy->getTransmitterAddress());
    ctrl->setDest(frameCopy->getReceiverAddress());
    ctrl->setFrmPLength(frameCopy->getFrmPLength());
    frameCopy->setControlInfo(ctrl);
    // (LC)
    //
    sfSentCount.record(presentSF);
    tpSentCount.record(presentTP);
    dcTimeOff = true;
    txDeferred = true;
    scheduleAt(simTime() + calTimeOff(presentSF, presentBW, presentFrmPLength + 13 , 1 /* use CRC */, 4 /* CR */), endDCTimeOff);
    // note: FrmPLength + 13,  13 -> overhead due to FRMheader + FPort, MacHeader and MIC
    emit(LoRa_MacFrameSent, (int) frameCopy->getMType());
    if (frameCopy->getMType() == CONF_DATA_UP)
        numSentConFrames++;
    if (frameCopy->getMType() == UCONF_DATA_UP)
        numSentUnconFrames++;
    sendDown(frameCopy);
}

void LoRaMac::sendAckFrame()    // (LC) not the real LoRa ACk frame. Need to be updated
{
    EV << "sending Ack frame\n";
    auto ackFrame = new CsmaCaMacAckFrame("CsmaAck");
    ackFrame->setByteLength(ackLength);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    sendDown(ackFrame);
}

/****************************************************************
 * Helper functions.
 */
void LoRaMac::finishCurrentTransmission()
{
    // (LC)
    scheduleAt(simTime() + waitDelay1Time, endDelay_1);
    scheduleAt(simTime() + waitDelay1Time + listening1Time, endListening_1);
    // (LC)
    //scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time, endDelay_2);
    //scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time + listening2Time, endListening_2);
    // (LC)
    //
    scheduleAt(simTime() + waitDelay1Time + waitDelay2Time, endDelay_2);
    scheduleAt(simTime() + waitDelay1Time + waitDelay2Time + listening2Time, endListening_2);

    if ( !ACKPending )
    {
         popTransmissionQueue();
    }
}

// (LC)
void LoRaMac::setRetransmission()
{
    if ( ACKPending == true )
    {
        countRetry++;
        if (countRetry < maxRetry)
        {
            if ( sfMode == SFM0 || sfMode == SFM2 || sfMode == SFM3 )
            {
                if ((presentSF < 12 ) && ((countRetry % 2) == 0))
                {
                     presentSF++;
                     getCurrentTransmission()->setLoRaSF(presentSF);
                }
                else if ( sfMode == SFM2)
                     {
                         if ( (presentSF == 12 ) && ((countRetry % 2) == 0)  )
                         {
                             presentSF=7;
                             getCurrentTransmission()->setLoRaSF(presentSF);
                         }
                 }
            }
            if ( sfMode == SFM1)
                  {
                  }

            // set acktimeoutTime to a random into 1 to 3
            acktimeoutTime = intuniform(1,3);
            retry = false;
            txDeferred = true;
            scheduleAt(simTime() + acktimeoutTime, retransmission);
        }
        else
        {
             retry = false;
             ACKPending = false;
             countRetry = 0;
             // (LC) count frame ack failed
             if (simTime() >= getSimulation()->getWarmupPeriod()) {
                 numAckNotReceived++;
                 numGivenUp++;
             }
             // delete frame from queue
             popTransmissionQueue();
             EV << "Max Retries. Drop frame\n";
        }
    }
    // (LC)  TODO check
    // No frame received in neither receive window RX1 nor RX2
    if(evaluateADRinNode)
    {
         ADR_ACK_CNT++;
         if(ADR_ACK_CNT == ADR_ACK_LIMIT) sendNextPacketWithADRACKReq = true;
         if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY)
         {
             ADR_ACK_CNT = 0;
             increaseSFIfPossible();
         }
    }
}

void LoRaMac::manageFrame(LoRaMacFrame *frame)
{
    // (LC)
    // TODO check if received seq is correct
    sequenceNumberRemote = frame->getSequenceNumber();
    if ( frame->getACK() == true )
    {
        ACKPending = false;
        retry = false;
        if ( sfMode == SFM3)
        {
            if ( presentSF > 7 )
            {
               presentSF--;
            }
        }
        if (countRetry == 0) {
            if (simTime() >= getSimulation()->getWarmupPeriod())
                numAckReceivedNoRetry++;
        } else {
            countRetry = 0;
        }
        if (simTime() >= getSimulation()->getWarmupPeriod()) {
            numAckReceived++;
            rcvdAckPk.record(1);
        }
        popTransmissionQueue();
        EV << "Received Ack frame\n";
    };
    // (LC) has  been moved from SimpleLoRaApp
    //
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT = 0;
//        if(frame->getMType() == UCONF_DATA_DW)    //  (LC) to REVIEW
        if ( frame->getFOpsLen() != 0 ) {   // we only assume LinkADRreq command
            {
                numADRCommandReceived++;
                if(frame->getOptions().getLoRaTP() != -1)
                {
                    presentTP = frame->getOptions().getLoRaTP();
                }
                if(frame->getOptions().getLoRaSF() != -1)
                {
                    presentSF = frame->getOptions().getLoRaSF();
                }
                // TODO: consider to answer a possible LinkADRreq after receiving that command
            }
        }
    }
}

LoRaMacFrame *LoRaMac::getCurrentTransmission()
{
    return static_cast<LoRaMacFrame*>(transmissionQueue.front());
}

void LoRaMac::popTransmissionQueue()
{
    EV << "Dropping frame from transmission queue\n";
    delete transmissionQueue.pop();
    // (LC)
    txQueueFull = false;
    txQueueEmpty = transmissionQueue.isEmpty();

    if (queueModule) {
        // tell queue module that we've become idle
        EV << "requesting another frame from queue module\n";
        queueModule->requestPacket();
    }
}

bool LoRaMac::isReceiving()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_RECEIVING;
}

bool LoRaMac::isAck(LoRaMacFrame *frame)
{
    // return dynamic_cast<LoRaMacFrame *>(frame);
    return frame->getACK();
}

bool LoRaMac::isBroadcast(LoRaMacFrame *frame)
{
    return frame->getReceiverAddress().isBroadcast();
}

bool LoRaMac::isForUs(LoRaMacFrame *frame)
{
    return frame->getReceiverAddress() == address;
}

void LoRaMac::turnOnReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
}

void LoRaMac::turnOffReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

DevAddr LoRaMac::getAddress()
{
    return address;
}

// (LC)
//
 double LoRaMac::getPresentTP()
 {
     return presentTP;
 }

 int LoRaMac::getPresentSF()
 {
     return presentSF;
 }

 Hz LoRaMac::getPresentCF()
  {
      return presentCF;
  }

 Hz LoRaMac::getPresentBW()
  {
      return presentBW;
  }

 simtime_t LoRaMac::getTimeOff(int fplen)
  {
     return calTimeOff(presentSF, presentBW, fplen + 13, 1 /* use CRC */, 4 /* CR */);
  }

 simtime_t LoRaMac::calTimeOff(int sf, Hz bw, int pl, int crc, int cr )
 {
 /* Get time interval in which only can be one transmission, due to duty cycle restriction
 // Parameters:
 // sf : SF value (7..12)
 // bw : bandwidth (Hz, 125000 or 250000)
 // pl : physical payload (bytes), macheader (1) + frameheader (7) + FPort (1) + MIC (4)
 // crc: use physical crc or not, (1 , 0); standard use: UL -> CRC, DL -> no CRC
 // cr: coding rate: (1..4)
 */
         simtime_t time;
         // for PL = 20, CR = 4, crc = 1, bw = 125 kHz, de, H = 0 (explicit phy header)
         // DC = 1%
         // if(presentSF == 7) time = 7.808;
         // if(presentSF == 8) time = 13.9776;
         // if(presentSF == 9) time = 24.6784;
         // if(presentSF == 10) time = 49.3568;
         // if(presentSF == 11) time = 98.7136;  // 85.6064;
         // if(presentSF == 12) time = 171.2128;

         // other parameters
         int h = 0;         // implicit header (0,1); 0 -> explicit header
         double dc = 0.01;  // duty cycle 1%
         int de = 0;        // 0 for SF7 - SF10, 1 for SF11 - SF12
         int npreamb = 8;   // num preamble symbols
         // vars
         double ts;
         simtime_t tf;
         simtime_t tpreamb;
         simtime_t toa; // Time on Air
         int nsymb;

         ts = pow(2,sf) / bw.get() ;    //125000; // double(bw);

         if ( sf == 11 || sf == 12 ) de = 1;
         nsymb = 8 + std::max( int(ceil(( 8.0*pl - 4*sf + 28 + 16*crc - 20*h)/( 4*(sf-2*de))) * (cr+4)), 0 );
         tf = nsymb * ts;
         tpreamb = ( npreamb + 4.25 ) * ts;
         toa = tpreamb + tf;
         time = toa / dc;
         return time;
 }

 void LoRaMac::increaseSFIfPossible()
 {
      if(presentSF < 12) presentSF++;
 }


} // namespace inet
