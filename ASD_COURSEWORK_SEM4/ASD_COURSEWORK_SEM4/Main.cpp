#include <iostream>
#include "Transport_Network_class.h"

using namespace std;

int main()
{
	cout << "Example of the first transport network.\n";
	Transport_Network firstNetwork = Transport_Network("firstNetwork.txt");
	cout << "List of transport network vertices with their indexes:\n";
	cout << firstNetwork.getNetworkVertices();
	cout << "\nTransport network adjacency matrix:\n";
	cout << firstNetwork.getMatrixAdjacency().toString();
	cout << "\nMatrix max flow and value max flow in the transport network:\n";
	cout << firstNetwork.findMaxFlow();
	cout << "\n\n";

	cout << "\nExample of the second transport network.\n";
	Transport_Network secondNetwork = Transport_Network("secondNetwork.txt");
	cout << "List of transport network vertices with their indexes:\n";
	cout << secondNetwork.getNetworkVertices();
	cout << "\nTransport network adjacency matrix:\n";
	cout << secondNetwork.getMatrixAdjacency().toString();
	cout << "\nMatrix max flow and value max flow in the transport network:\n";
	cout << secondNetwork.findMaxFlow();
	cout << "\n";

	return 0;
}