#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <list>
#include <algorithm>
#include <regex>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../3rdParty/AerialBase/include/AerialBase.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include <boost/filesystem.hpp>


using namespace std;

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 CGAL_Point;
typedef K::Vector_3 CGAL_Vector3;
template <class K> class CGAL_FaceT : public K::Triangle_3
{
public:
	CGAL_FaceT(CGAL_Point p0, CGAL_Point p1, CGAL_Point p2) :K::Triangle_3(p0, p1, p2)
	{
		index = 0;
	}
	size_t index;
};
typedef CGAL_FaceT<K> CGAL_Face;
typedef CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_triangle_primitive<K, list<CGAL_FaceT<K>>::iterator>>> Tree;
typedef cv::Point3d Vector; //向量
typedef Vector NormalVector; //法向量

struct CBackProjectConfig
{
	bool IsDoBlockJudgment = true;
	string ObjDir = "";
	string SvpPath = "";
	string BinSavePath = "";
	double CentralX = 0, CentralY = 0, CentralZ = 0;
	size_t ThisComputerIndex = 1, ComputersNum = 1;
};
class CFace3D
{
public:
	size_t Vertex1ID, Vertex2ID, Vertex3ID; //三角面片的顶点下标
	CGAL_Point FaceCenter;
	CFace3D()
	{
		Vertex1ID = Vertex2ID = Vertex3ID = 0;
		FaceCenter = CGAL_Point(0, 0, 0);
	}
	CFace3D(size_t Vertex1ID, size_t Vertex2ID, size_t Vertex3ID, CGAL_Point FaceCenter)
	{
		this->Vertex1ID = Vertex1ID;
		this->Vertex2ID = Vertex2ID;
		this->Vertex3ID = Vertex3ID;
		this->FaceCenter = FaceCenter;
	}

};
class CModel
{
public:
	vector<cv::Point3d> Vertex; //点
	vector<CFace3D> Face; //面片
	double CentralX, CentralY, CentralZ; //中心化坐标
	Tree CGALModelTree;

	CModel()
	{
		Vertex.resize(0);
		Face.resize(0);
		CentralX = CentralY = CentralZ = 0;
		FaceList.clear();
		CGALModelTree.clear();
	}
	void LoadFromDir(CBackProjectConfig& config)
	{
		this->CentralX = config.CentralX;
		this->CentralY = config.CentralY;
		this->CentralZ = config.CentralZ;

		vector<string> FilePaths;
		GetAllFormatFiles(config.ObjDir, ".obj", FilePaths);
		cout << "==> Found " << FilePaths.size() << " obj models!" << endl;
		for (size_t i = 0; i < FilePaths.size(); i++)
		{
			size_t OriginVertexNum = Vertex.size(), OriginFaceNum = Face.size(), VerticesNum = 0, FacesNum = 0;
			LoadObjFile(FilePaths[i], VerticesNum, FacesNum);
			
			size_t pos = FilePaths[i].find_last_of("/");
			string FileName = (pos == string::npos ? FilePaths[i] : FilePaths[i].substr(pos + 1));
			cout << "===> (" << i + 1 << "/" << FilePaths.size() << ") The model " << FileName << " contains " << VerticesNum << " vertices and " << FacesNum << " face slices!" << endl;
		}
		cout << endl << "==> Building CGAL tree..." << endl;
		mutex FaceList_Mutex;
#pragma omp parallel for
		for (long long i = 0; i < Face.size(); i++)
		{
			CGAL_Point p1(Vertex[Face[i].Vertex1ID].x, Vertex[Face[i].Vertex1ID].y, Vertex[Face[i].Vertex1ID].z);
			CGAL_Point p2(Vertex[Face[i].Vertex2ID].x, Vertex[Face[i].Vertex2ID].y, Vertex[Face[i].Vertex2ID].z);
			CGAL_Point p3(Vertex[Face[i].Vertex3ID].x, Vertex[Face[i].Vertex3ID].y, Vertex[Face[i].Vertex3ID].z);
			if (!CGAL::collinear(p1, p2, p3))//如果三点不共线...
			{
				FaceList_Mutex.lock();
				FaceList.emplace_back(p1, p2, p3);
				FaceList_Mutex.unlock();
			}
		}
		CGALModelTree.insert(FaceList.begin(), FaceList.end());
		CGALModelTree.accelerate_distance_queries();
	}

private:
	list<CGAL_FaceT<K>> FaceList;

	void GetAllFormatFiles(string DirPath, string Format, vector<string>& FilePaths)
	{
		boost::filesystem::path Folder(DirPath);
		if (boost::filesystem::exists(Folder) && boost::filesystem::is_directory(Folder))
		{
			boost::filesystem::recursive_directory_iterator iter(Folder);
			boost::filesystem::recursive_directory_iterator end;

			while (iter != end)
			{
				if (boost::filesystem::is_regular_file(*iter) && iter->path().extension() == Format)
				{
					string Path = iter->path().string();
					Path = regex_replace(Path, regex("\\\\"), "/");
					replace(Path.begin(), Path.end(), '\\', '/');
					FilePaths.push_back(Path);
				}
				iter++;
			}
		}
		else
		{
			cout << DirPath << " folder does not exist or is not a directory!" << endl;
			exit(0);
		}
	}
	void LoadObjFile(string ObjFilePath, size_t& VerticesNum, size_t& FacesNum)
	{
		ifstream file(ObjFilePath);
		string line;
		size_t InitialVertexIDOffset = Vertex.size();
		size_t InitialFaceIDOffset = Face.size();
		VerticesNum = FacesNum = 0;
		while (getline(file, line))
		{
			if (line.size() <= 4)continue;
			if (line[0] == 'v' && line[1] == ' ')
			{
				char v;
				double x, y, z;
				istringstream stream(line);
				stream >> v >> x >> y >> z;
				x += CentralX;
				y += CentralY;
				z += CentralZ;
				Vertex.emplace_back(x, y, z);
				VerticesNum++;
			}
			else if (line[0] == 'f' && line[1] == ' ')
			{
				char f, slash;
				size_t ID1, ID2, ID3, Dummy;
				istringstream stream(line);
				stream >> f >> ID1 >> slash >> Dummy >> ID2 >> slash >> Dummy >> ID3 >> slash >> Dummy;
				ID1 = ID1 - 1 + InitialVertexIDOffset;
				ID2 = ID2 - 1 + InitialVertexIDOffset;
				ID3 = ID3 - 1 + InitialVertexIDOffset;
				Face.emplace_back(ID1, ID2, ID3, CGAL_Point(0, 0, 0));
				FacesNum++;
			}
		}
#pragma omp parallel for
		for (long long i = InitialFaceIDOffset; i < InitialFaceIDOffset + FacesNum; i++)
		{
			cv::Point3d P1 = Vertex[Face[i].Vertex1ID];
			cv::Point3d P2 = Vertex[Face[i].Vertex2ID];
			cv::Point3d P3 = Vertex[Face[i].Vertex3ID];
			Face[i].FaceCenter = CGAL_Point((P1.x + P2.x + P3.x) / 3.0, (P1.y + P2.y + P3.y) / 3.0, (P1.z + P2.z + P3.z) / 3.0);
		}
		file.close();
	}
};


void LoadBackProjectConfig(string ConfigPath, CBackProjectConfig& BackProjectConfig);
void Parse(string line, string& Config, string& value);
bool IsBlocked(CModel& Model, CAerialPhoto* Image, size_t FaceID);
void BackProject(cv::Point3d& Point3D, CAerialPhoto* Photo, cv::Point2d& Point2D);
bool IsValidBackProject(cv::Point2d& BackProjectP1, cv::Point2d& BackProjectP2, cv::Point2d& BackProjectP3, CAerialPhoto* Photo);
void ShowProgress(atomic<size_t>* Count, size_t* Total);
size_t GetMatchedNum(unordered_set<size_t>& Set1, unordered_set<size_t>& Set2);

int main(int argc, char* argv[])
{
	cout << "Reading config file and .svp file..." << endl;
	CBackProjectConfig BackProjectConfig;
	LoadBackProjectConfig("BackProjectConfig.ini", BackProjectConfig);

	CAerialBlock block;
	block.Reset();
	if (!block.LoadFromFile(BackProjectConfig.SvpPath.c_str()))
	{
		cout << "Unable to open .svp file!" << endl;
		exit(0);
	}

	size_t ImagesNum = block.GetNumPhoto();
	size_t ImagesPerComputer = ImagesNum / BackProjectConfig.ComputersNum + 1;
	size_t MinImageID = (BackProjectConfig.ThisComputerIndex - 1) * ImagesPerComputer;
	size_t MaxImageID = min(BackProjectConfig.ThisComputerIndex * ImagesPerComputer - 1, ImagesNum - 1);
	cout << "ImagesNum: " << ImagesNum << endl;
	cout << "ImagesPerComputer: " << ImagesPerComputer << endl;
	cout << "MinImageID: " << MinImageID << endl;
	cout << "MaxImageID: " << MaxImageID << endl;

	cout << "Loading .obj models..." << endl;
	CModel Model;
	Model.LoadFromDir(BackProjectConfig);
	cout << "All models have been read!" << endl;
	size_t VertexNum = Model.Vertex.size();
	size_t FaceNum = Model.Face.size();
	size_t CalculateImagesNum = MaxImageID - MinImageID + 1;
	size_t ViewNum = FaceNum * CalculateImagesNum;
	cout << "VertexNum: " << VertexNum << endl;
	cout << "FaceNum: " << FaceNum << endl;
	cout << "The current computer needs to calculate " << ViewNum << " views!" << endl;

	vector<unordered_set<size_t>> VisibleFaceIDs(CalculateImagesNum);
	atomic<size_t> BlockedViewNum = 0;
	atomic<size_t> CompletedViewNum = 0;
	cout << endl << (BackProjectConfig.IsDoBlockJudgment ? "Performing back projection (remove occlusion)..." : "Performing back projection (without remove occlusion)...") << endl;
	thread ShowBackProjectProgressThread(ShowProgress, &CompletedViewNum, &ViewNum);
	ShowBackProjectProgressThread.detach();
#pragma omp parallel for
	for (long long ImageID = MinImageID; ImageID <= MaxImageID; ImageID++)
	{
		CAerialPhoto* Image = block.GetPhoto(ImageID);
		const CCamera* ImageCamera = Image->GetCamera();
		int Width, Height;
		CCCDCamera* CCDCamera = (CCCDCamera*)ImageCamera;
		CCDCamera->GetSensorSizeInPixel(Width, Height);
		Image->SetDimension(Width, Height);
		for (size_t FaceID = 0; FaceID < FaceNum; FaceID++)
		{
			if (BackProjectConfig.IsDoBlockJudgment && IsBlocked(Model, Image, FaceID))
			{
				BlockedViewNum++;
				continue;
			}
			cv::Point3d FaceP1 = Model.Vertex[Model.Face[FaceID].Vertex1ID];
			cv::Point3d FaceP2 = Model.Vertex[Model.Face[FaceID].Vertex2ID];
			cv::Point3d FaceP3 = Model.Vertex[Model.Face[FaceID].Vertex3ID];

			cv::Point2d BackProjectP1, BackProjectP2, BackProjectP3;
			BackProject(FaceP1, Image, BackProjectP1);
			BackProject(FaceP2, Image, BackProjectP2);
			BackProject(FaceP3, Image, BackProjectP3);

			if (IsValidBackProject(BackProjectP1, BackProjectP2, BackProjectP3, Image))
			{
				VisibleFaceIDs[ImageID - MinImageID].insert(FaceID);
			}
		}
		CompletedViewNum.fetch_add(FaceNum);
	}
	cout << endl << "Back projection completed!" << endl;
	if (BackProjectConfig.IsDoBlockJudgment)
	{
		cout << "In the current calculation range, " << BlockedViewNum.load() * 100.0 / ViewNum << "% of the view is blocked!" << endl;
	}

	cout << "Exporting back projection information to .bin file..." << endl;
	ofstream ofs(BackProjectConfig.BinSavePath, ios::binary);

	bool IsPartedBin = (BackProjectConfig.ComputersNum > 1);
	ofs.write((char*)&IsPartedBin, sizeof(IsPartedBin));
	ofs.write((char*)&MinImageID, sizeof(MinImageID));
	ofs.write((char*)&MaxImageID, sizeof(MaxImageID));

	for (size_t i = 0; i < CalculateImagesNum; i++)
	{
		size_t ThisImageVisibleFaceNum = VisibleFaceIDs[i].size();
		ofs.write((char*)&ThisImageVisibleFaceNum, sizeof(ThisImageVisibleFaceNum));

		size_t Count = 0;
		for (size_t FaceID : VisibleFaceIDs[i])
		{
			ofs.write((char*)&FaceID, sizeof(FaceID));
			Count++;
			if (Count % 1000 == 0)
			{
				cout << "\r" << i + 1 << "/" << CalculateImagesNum << " : " << Count << "/" << VisibleFaceIDs[i].size();
			}
		}
	}
	ofs.close();
	cout << endl << "Exporting completed!" << endl;

	if (IsPartedBin)
	{
		cout << "All steps of this program have been completed!" << endl;
		return 0;
	}

	cout << "Calculating and exporting match information..." << endl;
	unordered_map<size_t, unordered_map<size_t, size_t>> MatchedNum;
	ofs = ofstream("Match result.txt");
	ofs << "images num: " << ImagesNum << endl;
	for (size_t ImageID = 0; ImageID < ImagesNum; ImageID++)
	{
		cout << "\r" << ImageID + 1 << "/" << ImagesNum;
		string ImageName = block.GetPhoto(ImageID)->GetURL();
		replace(ImageName.begin(), ImageName.end(), '\\', '/');
		ofs << "*******************************************" << endl;
		for (size_t MatchedImageID = 0; MatchedImageID < ImagesNum; MatchedImageID++)
		{
			string MatchedImageName = block.GetPhoto(MatchedImageID)->GetURL();
			replace(MatchedImageName.begin(), MatchedImageName.end(), '\\', '/');
			if (MatchedImageID == ImageID)
			{
				ofs << ImageID + 1 << "(" << ImageName << ")-" << MatchedImageID + 1 << "(" << MatchedImageName << "):" << VisibleFaceIDs[ImageID].size() << endl;;
				continue;
			}
			if (MatchedImageID > ImageID)
			{
				size_t num = GetMatchedNum(VisibleFaceIDs[ImageID], VisibleFaceIDs[MatchedImageID]);
				MatchedNum[ImageID][MatchedImageID] = num;
				ofs << ImageID + 1 << "(" << ImageName << ")-" << MatchedImageID + 1 << "(" << MatchedImageName << "):" << num << endl;
			}
			else
			{
				ofs << ImageID + 1 << "(" << ImageName << ")-" << MatchedImageID + 1 << "(" << MatchedImageName << "):" << MatchedNum[MatchedImageID][ImageID] << endl;
			}
		}
	}
	ofs.close();
	cout << endl << "All steps of this program have been completed!" << endl;
	return 0;
}

void Parse(string line, string& Config, string& value)
{
	size_t pos = line.find('=');
	if (pos == string::npos)
	{
		cout << "Error! " << line << " does not have \"=\"" << endl;
		exit(0);
	}
	Config = line.substr(0, pos);
	value = line.substr(pos + 1);
}
void LoadBackProjectConfig(string ConfigPath, CBackProjectConfig& BackProjectConfig)
{
	ifstream file(ConfigPath);
	file.imbue(locale(""));
	if (!file)
	{
		cout << "Error! " << ConfigPath << " does not exist!" << endl;
		exit(0);
	}
	string line, config, value;
	getline(file, line); //[BackProjectConfig]

	getline(file, line); //DoBlockJudgment=1
	Parse(line, config, value);
	BackProjectConfig.IsDoBlockJudgment = (value == "1");

	getline(file, line); //ObjDir=...
	Parse(line, config, BackProjectConfig.ObjDir);

	getline(file, line); //SvpPath=...
	Parse(line, config, BackProjectConfig.SvpPath);

	getline(file, line); //BinSavePath=...
	Parse(line, config, BackProjectConfig.BinSavePath);

	getline(file, line); //CentralX=0
	Parse(line, config, value);
	BackProjectConfig.CentralX = stod(value);

	getline(file, line); //CentralY=0
	Parse(line, config, value);
	BackProjectConfig.CentralY = stod(value);

	getline(file, line); //CentralZ=0
	Parse(line, config, value);
	BackProjectConfig.CentralZ = stod(value);

	getline(file, line); //DistributedCompute=1/1
	Parse(line, config, value);
	size_t pos = value.find('/');
	if (pos == string::npos)
	{
		cout << "Error! " << value << " does not have \"/\"" << endl;
		exit(0);
	}
	BackProjectConfig.ThisComputerIndex = stoi(value.substr(0, pos));
	BackProjectConfig.ComputersNum = stoi(value.substr(pos + 1));

	file.close();
}
bool IsBlocked(CModel& Model, CAerialPhoto* Image, size_t FaceID)
{
	const CAerialPhoto::EXTORI* exPrameter = Image->GetExtOri();
	CGAL_Point ImagePos(exPrameter->POS[0], exPrameter->POS[1], exPrameter->POS[2]); //影像的Pos

	CGAL_Point FaceCenter = Model.Face[FaceID].FaceCenter;
	double Distance = sqrt((ImagePos.x() - FaceCenter.x())*(ImagePos.x() - FaceCenter.x()) +
		(ImagePos.y() - FaceCenter.y())*(ImagePos.y() - FaceCenter.y()) +
		(ImagePos.z() - FaceCenter.z())*(ImagePos.z() - FaceCenter.z()));
	double Delta = 0.02;
	CGAL_Point CalculatePoint((ImagePos.x() - FaceCenter.x()) * Delta / Distance + FaceCenter.x(),
		(ImagePos.y() - FaceCenter.y()) * Delta / Distance + FaceCenter.y(),
		(ImagePos.z() - FaceCenter.z()) * Delta / Distance + FaceCenter.z());
	return Model.CGALModelTree.do_intersect(K::Ray_3(CalculatePoint, ImagePos));
}
void BackProject(cv::Point3d& Point3D, CAerialPhoto* Photo, cv::Point2d& Point2D)
{
	Photo->ObjectCS2PixelCS(Point3D.x, Point3D.y, Point3D.z, Point2D.x, Point2D.y);
}
bool IsValidBackProject(cv::Point2d& BackProjectP1, cv::Point2d& BackProjectP2, cv::Point2d& BackProjectP3, CAerialPhoto* Photo)
{
	int Width, Height;
	CCCDCamera* CCDCamera = (CCCDCamera*)Photo->GetCamera();
	CCDCamera->GetSensorSizeInPixel(Width, Height);
	if (BackProjectP1.x >= 0 && BackProjectP1.x <= Width && BackProjectP1.y >= 0 && BackProjectP1.y <= Height
		&& BackProjectP2.x >= 0 && BackProjectP2.x <= Width && BackProjectP2.y >= 0 && BackProjectP2.y <= Height
		&& BackProjectP3.x >= 0 && BackProjectP3.x <= Width && BackProjectP3.y >= 0 && BackProjectP3.y <= Height)
		return true;
	else
		return false;
}
void ShowProgress(atomic<size_t>* Count, size_t* Total)
{
	while (true)
	{
		size_t Current = (*Count).load();
		if (Current >= *Total)return;
		cout << "\r" << setw(6) << fixed << setprecision(2) << Current * 100.0 / *Total << "%";
		this_thread::sleep_for(chrono::milliseconds(300));
	}
}
size_t GetMatchedNum(unordered_set<size_t>& Set1, unordered_set<size_t>& Set2)
{
	if (Set1.size() > Set2.size())
	{
		return GetMatchedNum(Set2, Set1);
	}
	size_t re = 0;
	for (size_t FaceID : Set1)
	{
		if (Set2.count(FaceID))
		{
			re++;
		}
	}
	return re;
}
