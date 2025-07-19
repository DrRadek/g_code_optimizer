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

constexpr double halfPi = M_PIl / 2.0;

struct VoxelAreaInfo {
	int size;
	Eigen::Vector3<int> start_index = Eigen::Vector3<int>(0,0,0);
	Eigen::Vector3<int> end_index;

	VoxelAreaInfo(int size) {
		this->end_index = Eigen::Vector3<int>(size - 1, size - 1, size - 1);
		this->size = size;
	}
};

struct AxisOrder {
	int xAxisOrder;
	int yAxisOrder;
	int zAxisOrder;
	bool isReverse;
};

struct AxisOrderWithRotationInfo : AxisOrder {
	double pitch;
	double yaw;
};

struct Inputs {
	string inputPath;
	int voxel_size;
	bool output_voxels = false;
	bool use_gpu = true;
	string outputPath = "";
};

void ShrinkBoundingBoxInAxis(unsigned int* vtable, VoxelAreaInfo& voxelAreaInfo, int xAxisOrder, int yAxisOrder, int zAxisOrder, bool isReverse = false) {
	// std::cout << "ShrinkBoundingBoxInAxis, axis order: " << xAxisOrder << yAxisOrder << zAxisOrder << "\n";

	int axisRemap[3];
	Eigen::Vector3<int> start = voxelAreaInfo.start_index;
	Eigen::Vector3<int> end = voxelAreaInfo.end_index;
	Eigen::Vector3<int> direction = Eigen::Vector3<int>(1, 1, 1);

	if (isReverse) {
		std::swap(start[xAxisOrder], end[xAxisOrder]);
		direction[xAxisOrder] = -1;
	}

	for (int x = start[xAxisOrder]; x != end[xAxisOrder] + direction[xAxisOrder]; x += direction[xAxisOrder]) {
		for (int y = start[yAxisOrder]; y != end[yAxisOrder] + direction[yAxisOrder]; y += direction[yAxisOrder]) {
			for (int z = start[zAxisOrder]; z != end[zAxisOrder] + direction[zAxisOrder]; z += direction[zAxisOrder]) {
				axisRemap[xAxisOrder] = x;
				axisRemap[yAxisOrder] = y;
				axisRemap[zAxisOrder] = z;

				// Voxels are X-up
				bool isVoxel = CudaVoxelizer::checkVoxel(axisRemap[0], axisRemap[1], axisRemap[2], voxelAreaInfo.size, vtable);

				if (isVoxel) {
					// Found a voxel, save new start/end index
					if (isReverse) {
						/*std::cout << "New end index is: " << std::min(x, voxelAreaInfo.end_index[xAxisOrder]);
						std::cout << "\n x is:" << x << "\n";*/
						voxelAreaInfo.end_index[xAxisOrder] = std::min(x, voxelAreaInfo.end_index[xAxisOrder]);
					}
					else {
						/*std::cout << "New start index is: " << std::max(x, voxelAreaInfo.start_index[xAxisOrder]);
						std::cout << "\n x is:" << x << "\n";*/
						voxelAreaInfo.start_index[xAxisOrder] = std::max(x, voxelAreaInfo.start_index[xAxisOrder]);
					}
					return;
				}
			}
		}
	}
}

// Function to drop empty areas
void ShrinkBoundingBox(unsigned int* vtable, VoxelAreaInfo& voxelAreaInfo) {
	// std::cout << "Shrinking bounding box...\n";
	// std::cout << "Old box size is:\n";
	// std::cout << voxelAreaInfo.start_index << "\n";
	// std::cout << voxelAreaInfo.end_index << "\n";

	constexpr AxisOrder shrinkAxisOrder[] = {
		{0, 1, 2, false}, // Z-up
		{1, 2, 0, false}, // X-up
		{2, 0, 1, false}, // Y-up
		{0, 1, 2, true},  // Z-down
		{1, 2, 0, true},  // X-down
		{2, 0, 1, true}   // Y-down
	};

	for (const auto& axis : shrinkAxisOrder) {
		ShrinkBoundingBoxInAxis(vtable, voxelAreaInfo, axis.xAxisOrder, axis.yAxisOrder, axis.zAxisOrder, axis.isReverse);
	}

	// std::cout << "New box size is:\n";
	// std::cout << voxelAreaInfo.start_index << "\n";
	// std::cout << voxelAreaInfo.end_index << "\n";
}

// depends on zAxisOrder
// zAxisOrder = 2 => z-up
// zAxisOrder = 1 => y-up
// zAxisOrder = 0 => x-up
int CalculateAreaAngled(unsigned int* vtable, VoxelAreaInfo &voxelAreaInfo, int xAxisOrder, int yAxisOrder, int zAxisOrder, bool isReverse = false) {
	int axisRemap[3];
	bool reverseX = false;
	bool reverseY = false;
	bool reverseZ = isReverse;

	int sum = 0;
	int sum2 = 0;
	int i = 0;
	int maxprintI = 0;

	Eigen::Vector3<int> start = voxelAreaInfo.start_index;
	Eigen::Vector3<int> end = voxelAreaInfo.end_index;
	Eigen::Vector3<int> direction = Eigen::Vector3<int>(1,1,1);

	if (isReverse) {
		std::swap(start[zAxisOrder], end[zAxisOrder]);
		direction[zAxisOrder] = -1;
	}

	for (int x = start[xAxisOrder]; x != end[xAxisOrder] + direction[xAxisOrder]; x += direction[xAxisOrder]) {
		for (int y = start[yAxisOrder]; y != end[yAxisOrder] + direction[yAxisOrder]; y += direction[yAxisOrder]) {

			int v = 0;
			int vxy = 0;
			for (int z = start[zAxisOrder]; z != end[zAxisOrder] + direction[zAxisOrder]; z += direction[zAxisOrder]) {
				axisRemap[xAxisOrder] = x;
				axisRemap[yAxisOrder] = y;
				axisRemap[zAxisOrder] = z;

				if (i < maxprintI) {
					std::cout << axisRemap[0] << " " << axisRemap[1] << " " << axisRemap[2] << "\n";
				}
				
				bool isVoxel = CudaVoxelizer::checkVoxel(axisRemap[0], axisRemap[1], axisRemap[2], voxelAreaInfo.size, vtable);

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
	std::cout << "suma podpor: " << sum << "\n";
	std::cout << "suma modelu: " << sum2 << "\n";

	return sum;
}

void StoreBetterResult(double&minAreaPitch, double& minAreaYaw, int &minArea, double resultPitch, double resultYaw, int result) {
	if (result < minArea) {
		minAreaYaw = resultYaw;
		minAreaPitch = resultPitch;
		minArea = result;

		std::cout << "vysledek je lepsi\n";
		std::cout << "Pitch a yaw:" << minAreaPitch << " " << minAreaYaw << "\n";
	}
	std::cout << minArea << "\n";
	std::cout << "\n";
}

// Finds the best rotation in 90 degrees and returns yaw + pitch of the rotation
std::pair<float, float> OptimizeRotation90Degrees(unsigned int* vtable, VoxelAreaInfo &voxelAreaInfo) {
	std::cout << "Optimalizace rotace...\n";

	int minArea = std::numeric_limits<int>::max();
	double minAreayaw = 0;
	double minAreapitch = 0;
	int result;

	constexpr AxisOrderWithRotationInfo axisOrder[] = {
		// xAxis, yAxis, zAxis, isReverse,     pitch,     yaw
		{   0,     1,     2,     false,        0.0,       0.0       }, // Z-up
		{   0,     1,     2,     true,         M_PIl,     0.0       }, // Z-down
		{   1,     2,     0,     false,        0.0,      -halfPi    }, // X-up
		{   1,     2,     0,     true,         0.0,       halfPi    }, // X-down
		{   2,     0,     1,     false,        halfPi,    0.0       }, // Y-up
		{   2,     0,     1,     true,        -halfPi,    0.0       }  // Y-down
	};

	for (const auto& axis : axisOrder) {
		double result = CalculateAreaAngled(
			vtable, voxelAreaInfo,
			axis.xAxisOrder, axis.yAxisOrder, axis.zAxisOrder,
			axis.isReverse);

		StoreBetterResult(minAreapitch, minAreayaw, minArea,
			axis.pitch, axis.yaw, result);
	}

	std::cout << "Minimalni pitch a yaw:" << minAreapitch << " " << minAreayaw << "\n";

	return { minAreapitch,minAreayaw };
}

bool HandleBoolInput(string input) {
	bool result;
	for (auto& c : input)
	{
		c = tolower(c);
	}

	if (input == "true") {
		result = true;
	}
	else if (input == "false") {
		result = false;
	}
	else {
		std::istringstream ss(input);
		if (!(ss >> result)) {
			throw invalid_argument("Neni typu bool (validni hodnoty: [true, false, 0, 1]).");
		}
	}

	return result;
}

void HandleInputs(int argc, char** argv, Inputs &inputs) {
	if (argc > 1) {
		// Handle file name input
		std::cout << "Vstupni soubor je: " << argv[1] << "\n";
		inputs.inputPath = argv[1];

		if (inputs.inputPath.ends_with(".stl")) {
			inputs.inputPath.replace(inputs.inputPath.length() - 4, 4, "");
		}
	}
	else {
		throw invalid_argument("Vstupni soubor nespecifikovan.");
	}

	if (argc > 2) {
		// Handle voxel size input
		std::istringstream ss(argv[2]);
		if (!(ss >> inputs.voxel_size) || inputs.voxel_size <= 0) {
			throw invalid_argument("Rozliseni site nespecifikovano musi byt cele kladne cislo.");
		}
	}
	else {
		throw invalid_argument("Rozliseni site nespecifikovano.");
	}

	if (argc > 3) {
		// Handle whether voxelized file should be outputted
		try {
			inputs.output_voxels = HandleBoolInput(argv[3]);
		}
		catch (invalid_argument& e) {
			string err = "Chyba pri cteni argumentu zda ulozit voxely: ";
			err += e.what();
			throw invalid_argument(err);
		}
	}

	if (argc > 4) {
		// Handle whether gpu should be used
		try {
			inputs.use_gpu = HandleBoolInput(argv[4]);
		}
		catch (invalid_argument& e) {
			string err = "Chyba pri cteni argumentu zda pouzit gpu: ";
			err += e.what();
			throw invalid_argument(err);
		}
	}

	if (argc > 5) {
		inputs.outputPath = argv[5];

		if (inputs.outputPath.ends_with(".stl")) {
			inputs.outputPath.replace(inputs.outputPath.length() - 4, 4, "");
		}
	}

	if (inputs.outputPath == "") {
		inputs.outputPath = inputs.inputPath + "-out";
	}
}

int main(int argc, char **argv) {
	Inputs inputs;

	try
	{
		HandleInputs(argc, argv, inputs);
	}
	catch (invalid_argument& e)
	{
		std::cerr << "Chyba pri cteni vstupu: " << e.what() << "\n";
		return -1;
	}

	string inputFile = inputs.inputPath + ".stl";
	string outputFileNameStl = inputs.outputPath + ".stl";
	string outputFileNameMatrix = inputs.outputPath + ".mat";

	string outputFileNameVox = inputs.outputPath + ".vox";
	string outputFileNameVox2 = inputs.outputPath + "2.vox";

	std::cout << "vstupni soubor: " << inputFile << "\n";
	std::cout << "vystupni soubor: " << outputFileNameStl << "\n";

	// Read stl
	ifstream readStream;
	readStream.open(inputFile, std::ios::binary);
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
	auto vtable = CudaVoxelizer::VoxelizeMesh(&themesh, inputs.voxel_size, !inputs.use_gpu, true, inputs.output_voxels ? outputFileNameVox : "");

	VoxelAreaInfo voxelAreaInfo = VoxelAreaInfo(inputs.voxel_size);
	ShrinkBoundingBox(vtable, voxelAreaInfo);
	// Optimize the rotation
	auto [pitch,yaw] = OptimizeRotation90Degrees(vtable, voxelAreaInfo);

	// Rotate the model
	Eigen::Quaternionf q =
		Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX());
	q.normalize();

	for (int i = 0; i < vertices.size(); i++) {
		vertices[i] = q * vertices[i];
		themesh.vertices[i] = (Vec3)vertices[i];
	}

	themesh.bbox.clear();
	themesh.clear_bbox();

	// Export the final voxel model
	if (inputs.output_voxels) {
		auto vtableFinal = CudaVoxelizer::VoxelizeMesh(&themesh, inputs.voxel_size, !inputs.use_gpu, true, outputFileNameVox2);
	}

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