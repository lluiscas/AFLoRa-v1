//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "LoRaGWMac.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"


namespace inet {

Define_Module(LoRaGWMac);

void LoRaGWMac::initialize(int stage)
{
    MACProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // subscribe for the information of the carrier sense
        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        //radioModule->subscribe(IRadio::radioModeChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radio = check_and_cast<IRadio *>(radioModule);
        waitingForDC = false;
        dutyCycleTimer = new cMessage("Duty Cycle Timer");
        const char *addressString = par("address");
        GW_forwardedDown = 0;
        GW_droppedDC = 0;
        GW_presentSF = 0;
        GW_presentTP = 0;
        // (LC)
        cGW_droppedDCVector.setName("GW Dropped DC");
        WATCH(GW_presentSF);
        WATCH(GW_presentTP);
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = DevAddr::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        radio->setRadioMode(IRadio::RADIO_MODE_TRANSCEIVER);
    }
}

void LoRaGWMac::finish()
{
    recordScalar("GW_forwardedDown", GW_forwardedDown);
    recordScalar("GW_droppedDC", GW_droppedDC);
    cancelAndDelete(dutyCycleTimer);
}


InterfaceEntry *LoRaGWMac::createInterfaceEntry()
{
    InterfaceEntry *e = new InterfaceEntry(this);

    // data rate
    //e->setDatarate(bitrate);

    // generate a link-layer address to be used as interface token for IPv6
    //e->setMACAddress(address);
    //e->setInterfaceToken(address.formInterfaceIdentifier());

    // capabilities
    //e->setMtu(par("mtu"));
    //e->setMulticast(true);
    //e->setBroadcast(true);
    //e->setPointToPoint(false);

    return e;
}

void LoRaGWMac::handleSelfMessage(cMessage *msg)
{
    if(msg == dutyCycleTimer) waitingForDC = false;
}

void LoRaGWMac::handleUpperPacket(cPacket *msg)
{
    if(waitingForDC == false)
    {
        LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
        frame->removeControlInfo();
        LoRaMacControlInfo *ctrl = new LoRaMacControlInfo();
        ctrl->setSrc(address);
        ctrl->setDest(frame->getReceiverAddress());
        GW_presentSF = frame->getLoRaSF();
        GW_presentTP = frame->getLoRaTP();
        frame->setControlInfo(ctrl);
        sendDown(frame);
        waitingForDC = true;

        // TODO (LC) doesn't consider SF orthogonality, destination node ... ?

        simtime_t delta;

// original settings in flora
        // for PL = 15 bytes, CR = 4, BW= 125kHz, CRC=yes, H=0 (explicit header)
        // DC = 10%

//        if(frame->getLoRaSF() == 7) delta = 0.61696;
//        if(frame->getLoRaSF() == 8) delta = 1.23392;
//        if(frame->getLoRaSF() == 9) delta = 2.14016;
//        if(frame->getLoRaSF() == 10) delta = 4.28032;
//        if(frame->getLoRaSF() == 11) delta = 8.56064;  //wrong;
//        if(frame->getLoRaSF() == 12) delta = 14.49984;

// new settings in aflora (and flora-v0.8-crl)
        // for PL = 15 bytes, CR = 4, BW= 125kHz, CRC=no, H=0 (explicit header)

        //  SF     ToA       DC 1%     DC 10%
        //   7     0.06169     6.1696   0.61696
        //   8     0.107      10.7008   1.07008
        //   9     0.214      21.4016   2.14016
        //  10     0.362      36.2496   3.62496
        //  11     0.725      72.4992   7.24992
        //  12     1.450     144.9984  14.49984

        // if(frame->getLoRaSF() == 7)  delta =  0.61696;
        // if(frame->getLoRaSF() == 8)  delta =  1.07008;
        // if(frame->getLoRaSF() == 9)  delta =  2.14016;
        // if(frame->getLoRaSF() == 10) delta =  3.62496;
        // if(frame->getLoRaSF() == 11) delta =  7.24992;
        // if(frame->getLoRaSF() == 12) delta = 14.49984;

// new ToA and DC computation
        delta = calTimeOff(GW_presentSF, frame->getLoRaBW(), frame->getFrmPLength() + 13, 0 /* no crc */, 4 /* CR */);
        scheduleAt(simTime() + delta, dutyCycleTimer);
        GW_forwardedDown++;
    }
    else
    {
        GW_droppedDC++;
        cGW_droppedDCVector.record(1);
        delete msg;
    }
}

void LoRaGWMac::handleLowerPacket(cPacket *msg)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
    if(frame->getReceiverAddress() == DevAddr::BROADCAST_ADDRESS)
        sendUp(frame);
    else
        delete frame;
}

void LoRaGWMac::sendPacketBack(LoRaMacFrame *receivedFrame)
{
    EV << "sending Data frame back" << endl;
    LoRaMacFrame *frameToSend = new LoRaMacFrame("BackPacket");
    frameToSend->setReceiverAddress(receivedFrame->getTransmitterAddress());
    sendDown(frameToSend);
}

void LoRaGWMac::createFakeLoRaMacFrame()
{

}

void LoRaGWMac::receiveSignal(cComponent *source, simsignal_t signalID, long value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            //transmission is finished
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
        }
        transmissionState = newRadioTransmissionState;
    }
}

DevAddr LoRaGWMac::getAddress()
{
    return address;
}

simtime_t LoRaGWMac::calTimeOff(int sf, Hz bw, int pl, int crc, int cr )
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

}
