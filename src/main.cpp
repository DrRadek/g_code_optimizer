#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <limits>

#include "OpenSTL/stl.h"

#include "stlTools.h"
#include "optimizer.h"

#include "Eigen/Core";
#include "Eigen/Geometry"

#include <math.h>
#include "cuda_voxelizer.h"

using namespace std;

using namespace trimesh;

using namespace Eigen;

// depends on zAxisOrder
// zAxisOrder = 2 => z-up
// zAxisOrder = 1 => y-up
// zAxisOrder = 0 => x-up
int CalculateAreaAngled(unsigned int* vtable, int size, int xAxisOrder, int yAxisOrder, int zAxisOrder, bool reverse = false) {

	int axisRemap[3];
	bool reverseX = false;
	bool reverseY = false;
	bool reverseZ = reverse;

	int sum = 0;

	int sum2 = 0;
	int startIndex = 0;
	int endIndex = size;
	int i = 0;
	int maxprintI = 0;
	for (int x = (reverseX ? size - 1 : startIndex); x != (reverseX ? -1 : endIndex); x += (reverseX ? -1 : 1)) {
		
		for (int y = (reverseY ? size - 1 : startIndex); y != (reverseY ? -1 : endIndex); y += (reverseY ? -1 : 1)) {
			int v = 0;
			int vxy = 0;

			
			for (int z = (reverseZ ? size - 1 : startIndex); z != (reverseZ ? -1 : endIndex); z += (reverseZ ? -1 : 1)) {
				axisRemap[xAxisOrder] = x;
				axisRemap[yAxisOrder] = y;
				axisRemap[zAxisOrder] = z;

				if (i < maxprintI) {
					std::cout << axisRemap[0] << " " << axisRemap[1] << " " << axisRemap[2] << "\n";
					
				}
				
				bool isVoxel = CudaVoxelizer::checkVoxel(axisRemap[0], axisRemap[1], axisRemap[2], size, vtable);
				if (isVoxel) {
					vxy += v;
					v = 0;
				}
				else {
					v += 1;
				}
				sum2 += 1;
			}
			if (i < maxprintI) {
				std::cout << "\n";
			}
			
			sum += vxy;
			i++;
		}
	}
	std::cout << "sum of model is: " << sum << "\n";
	std::cout << "max sum is: " << sum2 << "\n";

	return sum;
}


void StoreBetterResult(double&minAreaYaw, double&minAreaPitch, int &minArea, double resultYaw, double resultPitch, int result) {
	if (result < minArea) {
		minAreaYaw = resultYaw;
		minAreaPitch = resultPitch;
		minArea = result;

		std::cout << "result is better\n";
		
	}
	std::cout << minArea << "\n";
}

// Finds the best rotation in 90 degrees and returns yaw + pitch of the rotation
std::pair<float, float> OptimizeRotation90Degrees(unsigned int* vtable, int size) {

	int minArea = std::numeric_limits<int>::max();
	double minAreayaw = 0;
	double minAreapitch = 0;
	int result;

	// z-up
	result = CalculateAreaAngled(vtable, size, 0, 1, 2);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, 0, 0, result);
	// z-up flipped
	result = CalculateAreaAngled(vtable, size, 0, 1, 2, true);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, M_PIl, 0, result);

	// x-up
	result = CalculateAreaAngled(vtable, size, 1, 2, 0);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, M_PIl / 2.0, 0, result);
	// x-up flipped
	result = CalculateAreaAngled(vtable, size, 1, 2, 0, true);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, M_PIl / 2.0, M_PIl, result);

	// y-up
	result = CalculateAreaAngled(vtable, size, 2, 0, 1);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, 0, M_PIl / 2.0, result);
	// y-up flipped
	result = CalculateAreaAngled(vtable, size, 2, 0, 1, true);
	StoreBetterResult(minAreayaw, minAreapitch, minArea, M_PIl, M_PIl / 2.0, result);

	std::cout << "Min yaw and pitch:" << minAreayaw << " " << minAreapitch << "\n";

	return { minAreayaw,minAreapitch };
}

int main(int argc, char **argv) {
	// Read input
	string inputFileName = "";
	if (argc < 2) {
		std::cout << "Vstupni soubor nespecifikovan.\n";
	}
	else {
		std::cout << "Vstupni soubor je: " << argv[1] << "\n";
		inputFileName = argv[1];
	}

	if (inputFileName.ends_with(".stl")) {
		inputFileName.replace(inputFileName.length() - 4, 4, "");
	}

	string fileName = inputFileName + ".stl";
	string outputFileNameStl = inputFileName + "-out.stl";
	string outputFileNameMatrix = inputFileName + "-out.mat";

	string outputFileNameVox = inputFileName + "-out.vox";
	string outputFileNameVox2 = inputFileName + "-outfirst.vox";


	std::cout << "vstupni soubor: " << fileName << "\n";
	std::cout << "vystupni soubor: " << outputFileNameStl << "\n";

	// Read stl
	ifstream readStream;
	readStream.open(fileName, std::ios::binary);
	if (!readStream.is_open()) {
		std::cout << "Vstupni soubor se nepovedlo otevrit.";
		return -1;
	}

	std::vector<openstl::Triangle> triangles = openstl::deserializeStl(readStream);
	readStream.close();

	// Convert to vertices and faces
	auto [vertices, faces] = openstl::convertToVerticesAndFaces(triangles);

	TriMesh themesh;

	// copy vertices
	themesh.vertices.resize(vertices.size());
	for (int i = 0; i < vertices.size(); i++) {
		themesh.vertices[i] = (Vec3)vertices[i];
	}

	// copy faces
	themesh.faces.resize(faces.size());
	for (int i = 0; i < faces.size();i++) {
		themesh.faces[i] = faces[i];
	}

	// Voxelize the model
	int voxel_size = 50;
	auto vtable = CudaVoxelizer::VoxelizeMesh(&themesh, voxel_size, false, true, outputFileNameVox2);

	// Optimize the rotation
	auto [yaw,pitch] = OptimizeRotation90Degrees(vtable, voxel_size);

	// Rotate the model
	Eigen::Quaternionf q =
		Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY());
	q.normalize();

	for (int i = 0; i < vertices.size(); i++) {
		vertices[i] = q * vertices[i];
		themesh.vertices[i] = (Vec3)vertices[i];
	}

	// Export the final voxel model
	auto vtableFinal = CudaVoxelizer::VoxelizeMesh(&themesh, voxel_size, false, true, outputFileNameVox);

	// Save the matrix
	auto matrix = q.toRotationMatrix();
	std::cout << matrix << "\n";
	ofstream matrixWriteStream;
	matrixWriteStream.open(outputFileNameMatrix);
	matrixWriteStream << matrix;
	matrixWriteStream.close();
	
	// Save the rotated STL
	ofstream writeStream;
	writeStream.open(outputFileNameStl, std::ios::binary);
	openstl::serializeBinaryStl(openstl::convertToTriangles(vertices,faces), writeStream);
	writeStream.close();
}