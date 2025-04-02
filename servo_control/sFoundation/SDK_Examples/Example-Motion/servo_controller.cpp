//Required include files
#include <stdio.h>
#include <string>
#include <iostream>
#include <thread>
#include "pubSysCls.h"

using namespace sFnd;

#define ACC_LIM_RPM_PER_SEC	100000
#define MOVE_DISTANCE_CNTS	800	// 800 = 1 rev
#define TIME_TILL_TIMEOUT	10000

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
}

void moveMotor(INode &node, int nodeId, double velRpm, int moveDist, int moveCount, 
				bool backnforth, SysManager* myMgr) {
	for (int i = 0; i < moveCount; i++) {
		node.Motion.MoveWentDone();
		node.AccUnit(INode::RPM_PER_SEC);
		node.VelUnit(INode::RPM);
		node.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
		node.Motion.VelLimit = velRpm;

		int dist = moveDist;
		if (backnforth && (i % 2 == 1)) {
			printf("Here");
			dist *= -1;
		}

		printf("Node %d: Move %d -> %d counts\n", nodeId, i, dist);
		node.Motion.MovePosnStart(dist);

		double timeout = myMgr->TimeStampMsec() + 5000;
		while (!node.Motion.MoveIsDone()) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Node %d: Timeout!\n", nodeId);
				break;
			}
		}
		printf("Node %d: Move Done\n", nodeId);
	}
}

int main(int argc, char* argv[])
{
	msgUser("Motion Example starting. Press Enter to continue.\n");

	size_t portCount = 0;
	std::vector<std::string> comHubPorts;
	SysManager* myMgr = SysManager::Instance();

	try {
		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %lu SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
		}

		if (portCount <= 0) {
			printf("Unable to locate SC hub port\n");
			msgUser("Press any key to continue.\n");
			return -1;
		}

		myMgr->PortsOpen(portCount);
		IPort &myPort = myMgr->Ports(0);
		printf(" Port[0]: state=%d, nodes=%d\n", myPort.OpenState(), myPort.NodeCount());

		if (myPort.NodeCount() < 2) {
			printf("Expected at least 2 nodes connected, but found %zu\n", myPort.NodeCount());
			return -1;
		}

		INode &node0 = myPort.Nodes(0);
		INode &node1 = myPort.Nodes(1);

		node0.EnableReq(false);
		node1.EnableReq(false);
		myMgr->Delay(200);

		node0.Status.AlertsClear();
		node0.Motion.NodeStopClear();
		node0.EnableReq(true);

		node1.Status.AlertsClear();
		node1.Motion.NodeStopClear();
		node1.EnableReq(true);

		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
		while (!node0.Motion.IsReady() || !node1.Motion.IsReady()) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Error: Timeout while enabling nodes\n");
				return -1;
			}
		}

		std::thread t0(moveMotor, std::ref(node0), 0, 200, MOVE_DISTANCE_CNTS*5, 1, false, myMgr);
		std::thread t1(moveMotor, std::ref(node1), 1, 0, MOVE_DISTANCE_CNTS, 0, false, myMgr);

		t0.join();
		t1.join();

		node0.EnableReq(false);
		node1.EnableReq(false);
		myMgr->PortsClose();
	}
	catch (mnErr& theErr) {
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msgUser("Press any key to continue.\n");
		return 0;
	}

	msgUser("Press any key to continue.\n");
	return 0;
}
