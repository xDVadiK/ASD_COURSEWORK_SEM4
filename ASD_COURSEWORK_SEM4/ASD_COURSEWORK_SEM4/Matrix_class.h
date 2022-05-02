#pragma once
#include <iostream>

using namespace std;

// Class Matrix
class Matrix
{
private:

	int** matrix; // Two-dimensional array storing values for the adjacency matrix
	size_t numberVertices; // The number of vertices in the graph

	// Creating an empty adjacency matrix
	void creatingEmptyMatrix(int numberVertices)
	{
		matrix = new int* [numberVertices];
		for (int i = 0; i < numberVertices; i++)
		{
			matrix[i] = new int[numberVertices];
			for (int j = 0; j < numberVertices; j++)
			{
				matrix[i][j] = 0;
			}
		}
	}

public:

	// Constructor of the Matrix class
	Matrix(int numberVertices = 0)
	{
		creatingEmptyMatrix(numberVertices); // Creating an empty adjacency matrix
		this->numberVertices = numberVertices;
	}
	// Assignment Constructor of the Matrix class
	Matrix(const Matrix& other)
	{
		creatingEmptyMatrix(other.numberVertices);
		this->numberVertices = other.numberVertices;
		for (int i = 0; i < numberVertices; i++)
		{
			this->matrix[i] = new int[numberVertices];
			for (int j = 0; j < numberVertices; j++)
			{
				this->matrix[i][j] = other.matrix[i][j];
			}
		}
	}
	// The destructor of the Red Black Tree class
	~Matrix()
	{
		clear();
		numberVertices = 0;
	}

	// Getting a two-dimensional array of string values for the adjacency matrix
	int** getMatrix()
	{
		return matrix;
	}

	// Getting the number of vertices in the adjacency matrix
	int getNumberVertices()
	{
		return numberVertices;
	}

	// Getting the edge value of a graph from the adjacency matrix
	int getEdgeValue(int firstVertex, int secondVertex)
	{
		if ((firstVertex < 0 || firstVertex >= getNumberVertices()) || (secondVertex < 0 || secondVertex >= getNumberVertices()))
		{
			throw domain_error("An invalid index of the graph vertex was entered");
		}
		else
		{
			return matrix[firstVertex][secondVertex];
		}
	}

	// Adding a new edge value to the adjacency matrix
	void addNewEdge(int firstVertex, int secondVertex, int weigh)
	{
		if ((firstVertex < 0 || firstVertex >= getNumberVertices()) || (secondVertex < 0 || secondVertex >= getNumberVertices()))
		{
			throw domain_error("An invalid index of the graph vertex was entered");
		}
		else
		{
			matrix[firstVertex][secondVertex] = weigh;
		}
	};

	// Getting a matrix as a string
	string toString() {
		stringstream martixOutput;
		if (numberVertices == 0)
		{
			return "";
		}
		else
		{
			for (int i = 0; i < numberVertices; i++) {
				martixOutput << i << " : ";
				for (int j = 0; j < numberVertices; j++)
					martixOutput << matrix[i][j] << " ";
				martixOutput << "\n";
			}
			return martixOutput.str();
		}
	}

	// Clearing the adjacency matrix
	void clear()
	{
		for (int i = 0; i < numberVertices; i++)
		{
			delete[] matrix[i];
		}
		delete[] matrix;
		numberVertices = 0;
	};

	// Overloading the assignment operator
	Matrix& operator=(const Matrix& other)
	{
		if (this->matrix != nullptr)
		{
			this->clear();
		}
		creatingEmptyMatrix(other.numberVertices);
		this->numberVertices = other.numberVertices;
		for (int i = 0; i < numberVertices; i++)
		{
			this->matrix[i] = new int[numberVertices];
			for (int j = 0; j < numberVertices; j++)
			{
				this->matrix[i][j] = other.matrix[i][j];
			}
		}
		return *this;
	}

	// Overloading the comparison operator
	bool operator==(const Matrix& other)
	{
		if (this->numberVertices != other.numberVertices)
		{
			return false;
		}
		else
		{
			for (int i = 0; i < numberVertices; i++)
			{
				for (int j = 0; j < numberVertices; j++)
				{
					if (this->matrix[i][j] != other.matrix[i][j])
					{
						return false;
					};
				}
			}
			return true;
		}
	}
}; 