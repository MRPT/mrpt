#include <Eigen/Dense>
#include <iostream>

int main(int argc, char** argv)
{
	std::cout << "EIGEN_MAX_ALIGN_BYTES " << EIGEN_MAX_ALIGN_BYTES << "\n"
			  << "EIGEN_MAX_STATIC_ALIGN_BYTES " << EIGEN_MAX_STATIC_ALIGN_BYTES
			  << "\n";
	return 0;
}
