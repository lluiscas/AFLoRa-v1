package loranetwork.simulations;

import inet.applications.udpapp.UDPBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import loranetwork.LoRaPhy.LoRaMedium;
import loranetwork.LoraNode.LoRaNode;
import loranetwork.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaNetAnalysis
{
    parameters:
        int numberOfNodes = default(1);
        int numOfConNodes = default(1);  //(LC)
        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=355.824,356.832");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=167.328,88.704");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=208.24002,27.216002");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.64801,88.704");
        }
        internetCloud: InternetCloud {
            @display("p=83.664,27.216002");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=24.192001,27.216002");
        }
        nsRouter: Router {
            @display("p=138.09601,27.216002");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

