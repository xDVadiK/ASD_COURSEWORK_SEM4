#pragma once
#include <iostream>
#include <limits>
#include <string>
#include <sstream>
#include <fstream>
#include "Map_class.h"
#include "Matrix_class.h"

// Class transport network 
class Transport_Network
{
private:

    Matrix* MatrixAdjacency; // A pointer to the adjacency matrix that stores all the edges of the transport network with their capacity
    Matrix* MatrixParent; // Pointer to the parent matrix to restore the shortest way
    Matrix* MatrixFlow; // Pointer to the maximum flow matrix for the transport network
    string* arrayOfVertices; // List of all vertexes of the transport network
    Red_Black_Tree<string, int>* Map; // A pointer to an associative array in which all the vertices of the transport network and their indexes are stored

    // Filling in the adjacency matrix 
    void fillingAdjacencyMatrix(Matrix& MatrixAdjacency)
    {
        for (int i = 0; i < MatrixAdjacency.getNumberVertices(); i++)
        {
            for (int j = 0; j < MatrixAdjacency.getNumberVertices(); j++)
            {
                if (MatrixAdjacency.getEdgeValue(i, j) == 0 && i != j)
                    MatrixAdjacency.addNewEdge(i, j, INT_MAX); // Assigning "infinity" in the absence of an edge between vertices
            }
        }
    };

    // Filling in the parent matrix
    void fillingParentMatrix(Matrix& MatrixParent)
    {
        for (int i = 0; i < MatrixParent.getNumberVertices(); i++)
        {
            for (int j = 0; j < MatrixParent.getNumberVertices(); j++)
            {
                MatrixParent.addNewEdge(i, j, i);
            }
        }
    };

    // Filling in the adjacency matrix from a file
    void fillMatrixFromFile(string nameFile)
    {
        string directionalArc;
        string firstVertex;
        string secondVertex;
        string edgeThroughput;
        ifstream in(nameFile);
        if (in.is_open())
        {
            while (getline(in, directionalArc)) // Reading a line from a file
            {
                // Extracting information about the first city from a string read from a file
                firstVertex = directionalArc.substr(0, directionalArc.find(' '));
                directionalArc.erase(0, directionalArc.find(' ') + 1);
                if (!Map->find(firstVertex)) Map->insert(firstVertex, Map->get_size_tree());
                // Extracting information about the second city from the line read from the file
                secondVertex = directionalArc.substr(0, directionalArc.find(' '));
                directionalArc.erase(0, directionalArc.find(' ') + 1);
                if (!Map->find(secondVertex)) Map->insert(secondVertex, Map->get_size_tree());
            }
        }
        in.close();
        arrayOfVertices = new string[Map->get_size_tree()]; // Creating a list of all vertices of a graph of the desired size
        MatrixAdjacency = new Matrix(Map->get_size_tree()); // Creating an adjacency matrix of the desired size
        MatrixParent = new Matrix(Map->get_size_tree()); // Creating a parent matrix of the desired size
        MatrixFlow = new Matrix(Map->get_size_tree());
        Map->clear();
        in.open(nameFile);
        if (in.is_open())
        {
            while (getline(in, directionalArc))
            {
                // Extracting information about the first city from a string read from a file
                firstVertex = directionalArc.substr(0, directionalArc.find(' '));
                directionalArc.erase(0, directionalArc.find(' ') + 1);
                if (!Map->find(firstVertex))
                {
                    arrayOfVertices[Map->get_size_tree()] = firstVertex;
                    Map->insert(firstVertex, Map->get_size_tree());
                }
                // Extracting information about the second city from the line read from the file
                secondVertex = directionalArc.substr(0, directionalArc.find(' '));
                directionalArc.erase(0, directionalArc.find(' ') + 1);
                if (!Map->find(secondVertex))
                {
                    arrayOfVertices[Map->get_size_tree()] = secondVertex;
                    Map->insert(secondVertex, Map->get_size_tree());
                }
                // Extracting information about the cost of a direct flight
                edgeThroughput = directionalArc.substr(0, directionalArc.find(' '));
                // Entry into the adjacency matrix of the cost of the flight
                MatrixAdjacency->addNewEdge(*Map->Get_map_element(firstVertex)->value, *Map->Get_map_element(secondVertex)->value, stoi(edgeThroughput));
                directionalArc.erase(0, directionalArc.find(' ') + 1);
            }
            in.close();
        }
    }

    // The Floyd-Warshell algorithm
    void algorithmFloydWarshell(Matrix& MatrixAdjacency, Matrix& MatrixParent)
    {
        for (int k = 0; k < Map->get_size_tree(); k++)
        {
            for (int i = 0; i < Map->get_size_tree(); i++)
            {
                for (int j = 0; j < Map->get_size_tree(); j++)
                {
                    if (MatrixAdjacency.getEdgeValue(i, k) < INT_MAX && MatrixAdjacency.getEdgeValue(k, j) < INT_MAX)
                    {
                        if (MatrixAdjacency.getEdgeValue(i, j) > (MatrixAdjacency.getEdgeValue(i, k) + MatrixAdjacency.getEdgeValue(k, j)))
                        {
                            MatrixAdjacency.addNewEdge(i, j, (MatrixAdjacency.getEdgeValue(i, k) + MatrixAdjacency.getEdgeValue(k, j)));
                            MatrixParent.addNewEdge(i, j, k); // Saving the way
                        }
                    }
                }
            }
        }
    }

    // Restoring the way using the parent matrix
    void restoringTheWay(Matrix& MatrixAdjacency, Matrix& MatrixParent, int indexSource, int indexDrain, stringstream& way, int& minFlow)
    {
        if (MatrixParent.getEdgeValue(indexSource, indexDrain) != indexSource)
        {
            restoringTheWay(MatrixAdjacency, MatrixParent, indexSource, MatrixParent.getEdgeValue(indexSource, indexDrain), way, minFlow);
            restoringTheWay(MatrixAdjacency, MatrixParent, MatrixParent.getEdgeValue(indexSource, indexDrain), indexDrain, way, minFlow);
        }
        else
        {
            // Finding the minimum bandwidth of an edge on the found way
            if (MatrixAdjacency.getEdgeValue(indexSource, indexDrain) < minFlow) minFlow = MatrixAdjacency.getEdgeValue(indexSource, indexDrain);
            way << MatrixParent.getEdgeValue(indexSource, indexDrain) << " ";
        }
    };

    // Ford Fulkerson algorithm for finding the maximum flow in the transport network
    int algorithmFordFulkerson(Matrix& MatrixAdjacency, Matrix& MatrixParent, Matrix& MatrixFlow, int& maxFlow, int indexSource, int indexDrain)
    {
        Matrix CopyMatrixAdjacency = MatrixAdjacency; // Creating a copy of the adjacency matrix of the original transport network for the Ford Warshell algorithm
        Matrix CopyMatrixParent = MatrixParent; // Creating a copy of the parent matrix of the original transport network for the Ford Warshell algorithm
        fillingAdjacencyMatrix(CopyMatrixAdjacency); // Filling the adjacency matrix with "infinity" values for Floyd Warshell's algorithm
        algorithmFloydWarshell(CopyMatrixAdjacency, CopyMatrixParent); // Finding the way from the source to the drain
        if (CopyMatrixAdjacency.getEdgeValue(indexSource, indexDrain) == INT_MAX) // Checking whether there is a way from the source to the drain
        {
            return maxFlow;
        }
        else
        {
            // Restoring the found path from the source to the drain
            int minFlow = INT_MAX;
            stringstream way;
            restoringTheWay(CopyMatrixAdjacency, CopyMatrixParent, indexSource, indexDrain, way, minFlow);
            way << indexDrain << " ";
            string wayFromVertexIndexes = way.str();
            // Getting edges from a found way
            while (wayFromVertexIndexes != "")
            {
                string firstVertex = wayFromVertexIndexes.substr(0, wayFromVertexIndexes.find(' '));
                wayFromVertexIndexes.erase(0, wayFromVertexIndexes.find(' ') + 1);
                string secondVertex = wayFromVertexIndexes.substr(0, wayFromVertexIndexes.find(' '));
                if (secondVertex != "")
                {
                    // Updating edge throughput in the residual network
                    MatrixAdjacency.addNewEdge(stoi(firstVertex), stoi(secondVertex), MatrixAdjacency.getEdgeValue(stoi(firstVertex), stoi(secondVertex)) - minFlow);
                    MatrixAdjacency.addNewEdge(stoi(secondVertex), stoi(firstVertex), MatrixAdjacency.getEdgeValue(stoi(secondVertex), stoi(firstVertex)) + minFlow);
                    // Adding the minimum flow value to the flow matrix on a given path
                    MatrixFlow.addNewEdge(stoi(firstVertex), stoi(secondVertex), MatrixFlow.getEdgeValue(stoi(firstVertex), stoi(secondVertex)) + minFlow);
                }
            }
            maxFlow += minFlow; // Increasing the maximum flow value
            // Recursive call of the Ford Fulkerson algorithm to find the maximum flow in the remaining network
            algorithmFordFulkerson(MatrixAdjacency, MatrixParent, MatrixFlow, maxFlow, indexSource, indexDrain);
        }
    }

public:

    // Constructor of the Floyd Warshell Algorithm class
    Transport_Network(string nameFile)
    {
        Map = new Red_Black_Tree<string, int>();
        fillMatrixFromFile(nameFile); // Filling in the adjacency matrix from a file
        fillingParentMatrix(*MatrixParent); // Filling in the parent matrix 
    };
    // The destructor of the Floyd Warshell Algorithm class
    ~Transport_Network()
    {
        MatrixAdjacency->clear();
        MatrixParent->clear();
        MatrixFlow->clear();
        delete[] arrayOfVertices;
        Map->clear();
    };

    // Getting a list of all vertices of the transport network.
    string getNetworkVertices()
    {
        stringstream listVertices;
        for (int i = 0; i < Map->get_size_tree(); i++)
        {
            listVertices << i << ". " << arrayOfVertices[i] << "\n";
        }
        return listVertices.str();
    }

    // Getting the adjacency matrix
    Matrix getMatrixAdjacency()
    {
        return *MatrixAdjacency;
    };

    // Getting the parent matrix
    Matrix getMatrixParent()
    {
        return *MatrixParent;
    };

    // Getting the maximum flow of the transport network
    int findMaxFlow()
    {
        Matrix MatrixAdjacency = *this->MatrixAdjacency;
        Matrix MatrixParent = *this->MatrixParent;
        int maxFlowNetwork = 0;
        int indexSource = *Map->Get_map_element("S")->value; // Getting the source index of the transport network
        int indexDrain = *Map->Get_map_element("T")->value; // Getting the transport network drain index
        // Ford Fulkerson algorithm for finding the maximum flow in the transport network
        algorithmFordFulkerson(MatrixAdjacency, MatrixParent, *MatrixFlow, maxFlowNetwork, indexSource, indexDrain);
        cout << "\nMatrix flow in the network:\n";
        cout << MatrixFlow->toString();
        cout << "\nValue max flow in the network: ";
        return maxFlowNetwork;
    }
};