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

#include "LoRaTransmitter.h"
#include "inet/physicallayer/analogmodel/packetlevel/ScalarTransmission.h"
#include "LoRaModulation.h"

namespace inet {

namespace physicallayer {

Define_Module(LoRaTransmitter);

// (LC)
//
#define PAYLOADBYTES_ACK 15
#define PAYLOADBYTES_DATA 20


LoRaTransmitter::LoRaTransmitter() :
    FlatTransmitterBase()
{
}

void LoRaTransmitter::initialize(int stage)
{
    TransmitterBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        preambleDuration = 0.001; //par("preambleDuration");
        headerBitLength = par("headerBitLength");
        bitrate = bps(par("bitrate"));
        power = W(par("power"));
        modulation =  new LoRaModulation();
        carrierFrequency = Hz(par("carrierFrequency"));
        bandwidth = Hz(par("bandwidth"));
        LoRaTransmissionCreated = registerSignal("LoRaTransmissionCreated");

        if(strcmp(getParentModule()->getClassName(), "inet::physicallayer::LoRaGWRadio") == 0)
        {
            iAmGateway = true;
        } else iAmGateway = false;
    }
}

std::ostream& LoRaTransmitter::printToStream(std::ostream& stream, int level) const
{
    stream << "LoRaTransmitter";
    return FlatTransmitterBase::printToStream(stream, level);
}




const ITransmission *LoRaTransmitter::createTransmission(const IRadio *transmitter, const cPacket *macFrame, const simtime_t startTime) const
{
    TransmissionRequest *controlInfo = dynamic_cast<TransmissionRequest *>(macFrame->getControlInfo());
    //W transmissionPower = controlInfo && !std::isnan(controlInfo->getPower().get()) ? controlInfo->getPower() : power;
    bps transmissionBitrate = controlInfo && !std::isnan(controlInfo->getBitrate().get()) ? controlInfo->getBitrate() : bitrate;
    const_cast<LoRaTransmitter* >(this)->emit(LoRaTransmissionCreated, true);
    const LoRaMacFrame *frame = check_and_cast<const LoRaMacFrame *>(macFrame);

    int nPreamble = 8;
    simtime_t Tsym = (pow(2, frame->getLoRaSF()))/(frame->getLoRaBW().get()/1000);
    simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    int payloadBytes = 0;
    int crc = 0;

    // (LC) check calculation of payload Symbol.
    // to uses the actual frame length
    //
    if(iAmGateway) {
         payloadBytes = frame->getFrmPLength() + 12;    //PAYLOADBYTES_ACK; // 15;
         if (payloadBytes != 12)
              payloadBytes++;      // if frame payload is not empty, FPort is present
         crc = 0;
       }
    else {
         payloadBytes = frame->getFrmPLength() + 13;   //PAYLOADBYTES_DATA;
         crc = 1;
       }

    // (LC) take into account low and high SF
    //
    int lCR = frame->getLoRaCR();
    int lH = 0;    // explicit physical header indication
    int lSF = frame->getLoRaSF();
    int lDe = 0;    // optimization for low bit rates (SF11 and SF12 )
    if (lSF == 11 || lSF == 12)
        lDe = 1;
    else
        lDe = 0;
    double mc = ceil((8.0*payloadBytes - 4*lSF + 28 + 16*crc - 20*lH)/(4*(lSF-2*lDe)));
    int payloadSymbNb = 8 + math::max(mc*(lCR + 4), 0);

    // TODO: (LC) check Theader and Tpayload  computation
    //
    simtime_t Theader = 0.5 * (payloadSymbNb) * Tsym / 1000;
    simtime_t Tpayload = 0.5 * (payloadSymbNb) * Tsym / 1000;

    const simtime_t duration = Tpreamble + Theader + Tpayload;
    const simtime_t endTime = startTime + duration;
    IMobility *mobility = transmitter->getAntenna()->getMobility();
    const Coord startPosition = mobility->getCurrentPosition();
    const Coord endPosition = mobility->getCurrentPosition();
    const EulerAngles startOrientation = mobility->getCurrentAngularPosition();
    const EulerAngles endOrientation = mobility->getCurrentAngularPosition();
    if(!iAmGateway) {
        LoRaRadio *radio = check_and_cast<LoRaRadio *>(getParentModule());
        radio->setCurrentTxPower(frame->getLoRaTP());
    }
    return new LoRaTransmission(transmitter, macFrame, startTime, endTime, Tpreamble, Theader, Tpayload, startPosition, endPosition, startOrientation, endOrientation, mW(math::dBm2mW(frame->getLoRaTP())), frame->getLoRaCF(), lSF, frame->getLoRaBW(), lCR);
}

}

}
