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

//based on Ieee802154NarrowbandMac

package loranetwork.LoRa;

import inet.linklayer.csmaca.CsmaCaMac;

simple LoRaMac extends CsmaCaMac
{
    parameters:
        @signal[LoRa_MacFrameSent](type=long); // optional
        @statistic[LoRa_MacFrameSent](source=LoRa_MacFrameSent; record=count);
        bitrate = 250bps;
        double initialLoRaTP @unit(dBm) = default(14dBm);
        int initialLoRaSF = default(12);
        string sfModePar = default("SFM0");
        double initialLoRaCF @unit(Hz) = default(868MHz);
        double initialLoRaBW @unit(Hz) = default(125kHz);
        int initialLoRaCR = default(4);
        bool initialUseHeader = default(true);
    		bool evaluateADRinNode = default(false);

        @class(inet::LoRaMac);
        gates:
        	input upperMgmtIn;
        	output upperMgmtOut;
}
