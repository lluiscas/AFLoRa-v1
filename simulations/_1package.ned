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
network loRaNetworkTest
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        @display("bgb=525,436");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=190,292");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=174,189;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=472,269");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=473,27");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=473,202");
        }
        internetCloud: InternetCloud {
            @display("p=257,27");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=104,27");
        }
        nsRouter: Router {
            @display("p=365,27");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

