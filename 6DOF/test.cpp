#include <iostream>
#include <Eigen/Core>

int main() {
    
    // Define a 2x2 matrix with predefined values
    Eigen::Matrix2d matA;
    matA << 1, 2,
            3, 4;

    // Define another 2x2 matrix
    Eigen::Matrix2d matB;
    matB << 5, 6, 
            7, 8;

    // Print the matrices
    std::cout << "Matrix A:\n" << matA << "\n\n";
    std::cout << "Matrix A1:\n" << matA << "\n\n";
    std::cout << "Matrix B:\n" << matB << "\n\n";

    // Matrix addition
    Eigen::Matrix2d matSum = matA + matB;
    std::cout << "A + B:\n" << matSum << "\n\n";

};