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

#include "SimpleLoRaApp.h"
#include "inet/mobility/static/StationaryMobility.h"
namespace inet {

Define_Module(SimpleLoRaApp);

void SimpleLoRaApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        useACK = par("confirmedTx");           // (LC)
        frmPLength = par("frmPLength");        //
        sentPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");
        // (LC)
        receivedPackets = 0;

        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        do {
            timeToFirstPacket = par("timeToFirstPacket");
            EV << "Time to first Packet: " << timeToFirstPacket << endl;
            //if(timeToNextPacket < 5) error("Time to next packet must be grater than 3");
        } while(timeToFirstPacket <= 5);
        subModLoRaMac = check_and_cast<LoRaMac *>(getContainingNode(this)->getSubmodule("LoRaNic")->getSubmodule("mac"));
        sendMeasurements = new cMessage("sendMeasurements");
        scheduleAt(simTime()+timeToFirstPacket, sendMeasurements);
        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");
    }
}

std::pair<double,double> SimpleLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void SimpleLoRaApp::finish()
{
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("receivedPackets", receivedPackets);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg == sendMeasurements)
        {
            // (LC)
            if ( useACK )
                sendConfData();   // (LC)
            else
                sendUnConfData();
            //
            if (simTime() >= getSimulation()->getWarmupPeriod())
                sentPackets++;
            delete msg;
            if(numberOfPacketsToSend == 0 || sentPackets < numberOfPacketsToSend)
            {
                sendMeasurements = new cMessage("sendMeasurements");
                simtime_t time = subModLoRaMac->getTimeOff(frmPLength);
                do {
                    timeToNextPacket = par("timeToNextPacket");
                    EV_INFO << "Time to Next Packet: " << timeToNextPacket << endl;
                    //if(timeToNextPacket < 3) error("Time to next packet must be grater than 3");
                } while(timeToNextPacket <= time);
                scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
            }
        }
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        delete msg;
    }
}

void SimpleLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    if (simTime() >= getSimulation()->getWarmupPeriod()) {
        if (packet->getMsgType() == TXCONFIG)
           receivedADRCommands++;   // TODO (LC) this count all packets received from NS. To fix.
        receivedPackets++ ;
    }

}

bool SimpleLoRaApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void SimpleLoRaApp::sendJoinRequest()
{
    LoRaAppPacket *request = new LoRaAppPacket("joinRequest");
    request->setKind(JOIN_REQUEST);   // TODO: (LC)  kind or MsgType ?
    lastSentMeasurement = 1;
    request->setSampleMeasurement(lastSentMeasurement);
    //
    // TODO: complete join request function
    //

    //emit(LoRa_AppPacketSent, 42);
}

// (LC)
//
void SimpleLoRaApp::sendConfData()
{
    LoRaAppPacket *request = new LoRaAppPacket("ConfDataFrame");
    request->setMsgType(CONF_DATA_UP);
    lastSentMeasurement = 2937; //rand();
    request->setSampleMeasurement(lastSentMeasurement);

    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;

    cInfo->setMType(CONF_DATA_UP);    // (LC)
//    cInfo->setADR(false);             // review setADR()
    cInfo->setACK(false);
//    cInfo->setLoRaUseHeader(request->getOptions().getUseHeader());
    cInfo->setFPending(false);
    cInfo->setFrmPLength(frmPLength);
    request->setControlInfo(cInfo);
    send(request, "appOut");  // (LC) cut
    emit(LoRa_AppPacketSent, subModLoRaMac->getPresentSF() );
}

void SimpleLoRaApp::sendUnConfData()
{
    LoRaAppPacket *request = new LoRaAppPacket("UnConfDataFrame");
    request->setMsgType(UCONF_DATA_UP);
    lastSentMeasurement = rand();
    request->setSampleMeasurement(lastSentMeasurement);

    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setMType(UCONF_DATA_UP);     //  (LC)
//    cInfo->setADR(false);                        // TODO: review
    cInfo->setACK(false);
//    cInfo->setLoRaUseHeader(request->getOptions().getUseHeader());
    cInfo->setFPending(false);
    cInfo->setFrmPLength(frmPLength);
    request->setControlInfo(cInfo);
    send(request, "appOut");
    emit(LoRa_AppPacketSent, subModLoRaMac->getPresentSF());
}

} //end namespace inet
