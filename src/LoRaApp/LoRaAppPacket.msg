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

enum AppPacketType
{
    JOIN_REQUEST = 1;
    JOIN_REPLY = 2;
    UCONF_DATA_UP = 3;
    UCONF_DATA_DW = 4;
    CONF_DATA_UP = 5;
    CONF_DATA_DW = 6;
    TXCONFIG = 8;
}


packet LoRaAppPacket {
    int msgType @enum(AppPacketType);
    int sampleMeasurement;
    //LoRaOptions options;
}
