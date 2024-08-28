#include "Solution.h"

Solution::Solution(Input* input) {
	// Initialize the size information
	m_input = input;

	if(DEBUG_SOL)
		printf("New Solution I from Input: N = %d, M = %d\n", m_input->getN(), m_input->getM());

	// Dynamically allocate memory for I_ij
	X_ijk = new bool**[m_input->getN()];
	for(int i = 0; i < m_input->getN(); ++i) {
		X_ijk[i] = new bool*[m_input->getM()];
		for(int j = 0; j < m_input->getM(); ++j) {
			X_ijk[i][j] = new bool[m_input->getM()];
			for(int k = 0; k < m_input->getM(); ++k) {
				X_ijk[i][j][k] = 0;
			}
		}
	}
}

Solution::Solution(const Solution &other) {
	// Initialize the size information
	m_input = other.m_input;

	// Dynamically allocate memory for I_ij
	X_ijk = new bool**[m_input->getN()];
	for(int i = 0; i < m_input->getN(); ++i) {
		X_ijk[i] = new bool*[m_input->getM()];
		for(int j = 0; j < m_input->getM(); ++j) {
			X_ijk[i][j] = new bool[m_input->getM()];
			for(int k = 0; k < m_input->getM(); ++k) {
				X_ijk[i][j][k] = other.X_ijk[i][j][k];
			}
		}
	}
}

Solution& Solution::operator=(const Solution &other) {
	// Initialize the size information
	m_input = other.m_input;

	// Dynamically allocate memory for I_ij
	X_ijk = new bool**[m_input->getN()];
	for(int i = 0; i < m_input->getN(); ++i) {
		X_ijk[i] = new bool*[m_input->getM()];
		for(int j = 0; j < m_input->getM(); ++j) {
			X_ijk[i][j] = new bool[m_input->getM()];
			for(int k = 0; k < m_input->getM(); ++k) {
				X_ijk[i][j][k] = other.X_ijk[i][j][k];
			}
		}
	}

	return *this;
}

Solution::~Solution() {
	for(int i = 0; i < m_input->getN(); ++i) {
		for(int j = 0; j < m_input->getM(); ++j) {
			delete[] X_ijk[i][j];
		}
		delete[] X_ijk[i];
	}
	delete[] X_ijk;
}


// Prints this solution
void Solution::PrintSolution() {
	// Print solution
	printf("X_ijk:\n");
	for(int i = 0; i < m_input->getN(); ++i) {
		printf(" i = %d:  \n", i);
		for(int j = 0; j < m_input->getM(); ++j) {
			for(int k = 0; k < m_input->getM(); ++k) {
				printf("%d ", X_ijk[i][j][k]);
			}
			printf("\n");
		}
	}
}

// Assigns agent i to task j, updates i's position and traversed distance
void Solution::Update(int i, int j, int k) {
	// Need to update: I_ij, x_i, traversed-i
	if(DEBUG_SOL)
		printf("Assigning %d to %d for slot %d\n", i, j, k);

	// Clear all other assignments for k
	for(int k_i = 0; k_i < m_input->getM(); k_i++) {
		// Reset all I_ij for this i
		X_ijk[i][j][k_i] = false;
	}

	// Assign i to j for slot k
	X_ijk[i][j][k] = true;
}


/*
 * TODO: Determines the probability reward gained for the stored solution
 */
double Solution::Benchmark() {
	return 100.0;
}


// TODO: Determines if this is a valid assignment solution (doesn't break constraints)
bool Solution::ValidSolution() {
	/// Check each set of constraints
	bool valid = true;

	return valid;
}

