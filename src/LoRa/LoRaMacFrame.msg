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

//
// TODO generated message class
//

cplusplus {{
#include "../misc/DevAddr.h"
#include <inet/common/Units.h>
}}

class noncobject DevAddr;
class noncobject inet::units::values::W;
class noncobject inet::units::values::Hz;

namespace inet;

enum MacCmdId
{
    LINK_ADR_REQ_ID = 3;
    MAC_CMD_ID_NOT_USED = -1;
}

enum MacCmdLen
{
    NO_OPTIONS = 0;
 	LINK_ADR_REQ_LEN = 5;
 	LINK_ADR_ANS_LEN = 2;
 	MAC_CMD_LEN_NOT_USED = -1;
}

class LoRaOptions {
    int Cid @enum(MacCmdId) = MAC_CMD_ID_NOT_USED;
    double LoRaTP = -1;
    int LoRaSF = -1;
    double LoRaBW = -1;
//    double LoRaCF = -1;
//    int LoRaCR = -1;
//    bool UseHeader = true;
//    bool ADRACKReq = false;
}

packet LoRaMacFrame {
    DevAddr transmitterAddress;
    DevAddr receiverAddress;

    int MType;         // (LC)
    int sequenceNumber;
    bool ADR;          // (LC)
    bool ADRACKReq;    // (LC)
    bool ACK;          // (LC)
    int FOpsLen  = NO_OPTIONS;        // (LC)
    bool FPending;     // (LC)
    int FrmPLength;    // (LC)
    double LoRaTP;
    inet::units::values::Hz LoRaCF;
    int LoRaSF;
    inet::units::values::Hz LoRaBW;
    int LoRaCR;
    bool LoRaUseHeader;
    double RSSI;
    double SNIR;
    simtime_t tmstm;
    LoRaOptions options;  // (LC)
}
