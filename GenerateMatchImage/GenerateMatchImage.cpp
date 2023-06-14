#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <regex>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include "../3rdParty/AerialBase/include/AerialBase.h"

#include <boost/filesystem.hpp>
#include <boost/system/config.hpp>
#include <boost/system/error_code.hpp>

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


struct CGenerateMatchImageConfig
{
	string ObjDir = "";
	double CentralX = 0, CentralY = 0, CentralZ = 0;
	string SvpPath;
	bool IsDownSampling = true;
	bool IsMask = false;
	vector<pair<string, string>> TargetMatch;
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
class CFace2D
{
public:
	cv::Point2d P1, P2, P3;
	CFace2D()
	{
		P1 = P2 = P3 = cv::Point2d(0, 0);
	}
	CFace2D(cv::Point2d P1, cv::Point2d P2, cv::Point2d P3)
	{
		this->P1 = P1;
		this->P2 = P2;
		this->P3 = P3;
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
	void LoadFromDir(CGenerateMatchImageConfig& config)
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
					std::replace(Path.begin(), Path.end(), '\\', '/');
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
		while (std::getline(file, line))
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



void LoadGenerateMatchImageConfig(string ConfigPath, CGenerateMatchImageConfig& GenerateMatchImageConfig);
bool DoBackProject(CAerialBlock& block, CModel& Model, size_t ImageID, size_t FaceID, CFace2D& Face2D);
string GetFileName(string FilePath);
bool IsBlocked(CModel& Model, CAerialPhoto* Image, size_t FaceID);
void BackProject(cv::Point3d& Point3D, CAerialPhoto* Photo, cv::Point2d& Point2D);
bool IsValidBackProject(cv::Point2d& BackProjectP1, cv::Point2d& BackProjectP2, cv::Point2d& BackProjectP3, CAerialPhoto* Photo);
cv::Mat DownSampling(cv::Mat& input);
cv::Mat MergeTwoView(cv::Mat& Photo1, cv::Mat& Photo2, int GapWidth);
int main(int argc, char* argv[])
{
	cout << "Reading config file and .svp file..." << endl;
	CGenerateMatchImageConfig Config;
	LoadGenerateMatchImageConfig("GenerateMatchImageConfig.ini", Config);

	unordered_map<string, size_t> ImageName2ImageID;
	CAerialBlock block;
	block.Reset();
	if (!block.LoadFromFile(Config.SvpPath.c_str()))
	{
		cout << "Unable to open .svp file!" << endl;
		exit(0);
	}
	for (size_t ImageID = 0; ImageID < block.GetNumPhoto(); ImageID++)
	{
		string ImagePath = block.GetPhoto(ImageID)->GetURL();
		string ImageName = GetFileName(ImagePath);
		ImageName2ImageID[ImageName] = ImageID;
	}

	cout << "Loading .obj models..." << endl;
	CModel Model;
	Model.LoadFromDir(Config);
	cout << "All models have been read!" << endl;
	size_t VertexNum = Model.Vertex.size();
	size_t FaceNum = Model.Face.size();
	cout << "VertexNum: " << VertexNum << endl;
	cout << "FaceNum: " << FaceNum << endl;

	cout << "Calculate back projection relationships..." << endl;
	unordered_map<size_t, unordered_map<size_t, CFace2D>> ProjectInfo;

	size_t CurrentImagePair = 0;
	for (const pair<string, string>& pair : Config.TargetMatch)
	{
		cout << "\r" << CurrentImagePair + 1 << "/" << Config.TargetMatch.size();
		CurrentImagePair++;
		string LeftImageName = GetFileName(pair.first);
		string RightImageName = GetFileName(pair.second);
		if (ImageName2ImageID.find(LeftImageName) == ImageName2ImageID.end())
		{
			cout << "Image " << LeftImageName << " does not exists!" << endl;
			continue;
		}
		if (ImageName2ImageID.find(RightImageName) == ImageName2ImageID.end())
		{
			cout << "Image " << RightImageName << " does not exists!" << endl;
			continue;
		}
		size_t LeftImageID = ImageName2ImageID[LeftImageName], RightImageID = ImageName2ImageID[RightImageName];
		if (ProjectInfo.find(LeftImageID) == ProjectInfo.end())
		{
			ProjectInfo[LeftImageID] = unordered_map<size_t, CFace2D>();
			for (size_t FaceID = 0; FaceID < Model.Face.size(); FaceID++)
			{
				CFace2D Face2D;
				if (DoBackProject(block, Model, LeftImageID, FaceID, Face2D))
				{
					ProjectInfo[LeftImageID][FaceID] = Face2D;
				}
			}
		}
		if (ProjectInfo.find(RightImageID) == ProjectInfo.end())
		{
			ProjectInfo[RightImageID] = unordered_map<size_t, CFace2D>();
			for (size_t FaceID = 0; FaceID < Model.Face.size(); FaceID++)
			{
				CFace2D Face2D;
				if (DoBackProject(block, Model, RightImageID, FaceID, Face2D))
				{
					ProjectInfo[RightImageID][FaceID] = Face2D;
				}
			}
		}
	}
	cout << endl;

	CurrentImagePair = 0;
	for (const pair<string, string>& ImagePair : Config.TargetMatch)
	{
		string LeftImageName = GetFileName(ImagePair.first);
		string RightImageName = GetFileName(ImagePair.second);
		size_t LeftImageID = ImageName2ImageID[LeftImageName];
		size_t RightImageID = ImageName2ImageID[RightImageName];

		string LeftImagePath = block.GetPhoto(LeftImageID)->GetURL();
		LeftImagePath = regex_replace(LeftImagePath, regex("\\\\"), "/");
		std::replace(LeftImagePath.begin(), LeftImagePath.end(), '\\', '/');

		string RightImagePath = block.GetPhoto(RightImageID)->GetURL();
		RightImagePath = regex_replace(RightImagePath, regex("\\\\"), "/");
		std::replace(RightImagePath.begin(), RightImagePath.end(), '\\', '/');

		cv::Mat Photo1 = cv::imread(LeftImagePath);
		cv::Mat Photo2 = cv::imread(RightImagePath);

		if (Config.IsMask)
		{
			cv::Mat Mask1 = cv::Mat::zeros(Photo1.size(), CV_8UC3);
			cv::Mat Mask2 = cv::Mat::zeros(Photo2.size(), CV_8UC3);
			cv::Scalar Color(255, 255, 255);
			for (const pair<size_t, CFace2D>& pair : ProjectInfo[LeftImageID])
			{
				size_t FaceID = pair.first;
				if (ProjectInfo[RightImageID].find(FaceID) == ProjectInfo[RightImageID].end())
				{
					continue;
				}
				vector<cv::Point> pts;
				pts.push_back(pair.second.P1);
				pts.push_back(pair.second.P2);
				pts.push_back(pair.second.P3);
				polylines(Mask1, pts, true, Color, 1, 8, 0);//画三角形（不填充）
				fillConvexPoly(Mask1, pts, Color, 8, 0);//填充该三角形

				pts.clear();
				pts.push_back(ProjectInfo[RightImageID][FaceID].P1);
				pts.push_back(ProjectInfo[RightImageID][FaceID].P2);
				pts.push_back(ProjectInfo[RightImageID][FaceID].P3);
				polylines(Mask2, pts, true, Color, 1, 8, 0);
				fillConvexPoly(Mask2, pts, Color, 8, 0);
			}
			string OutputImageName = "Mask_" + to_string(CurrentImagePair + 1);
			cv::Mat TwoView;
			if (Config.IsDownSampling)
			{
				Mask1 = DownSampling(Mask1);
				Mask2 = DownSampling(Mask2);
				TwoView = MergeTwoView(Mask1, Mask2, 5);
				OutputImageName += "_DownSampling";
			}
			else
			{
				TwoView = MergeTwoView(Mask1, Mask2, 10);
			}
			OutputImageName += ".bmp";
			cv::imwrite(OutputImageName, TwoView);
			cout << "[" << CurrentImagePair + 1 << "/" << Config.TargetMatch.size() << "] Output mask: " << OutputImageName << endl;
		}
		else
		{
			srand((unsigned)time(NULL));
			float FillProbability = 1.0 / 20;//填充三角形的比例
			for (const pair<size_t, CFace2D>& pair : ProjectInfo[LeftImageID])
			{
				size_t FaceID = pair.first;
				if (ProjectInfo[RightImageID].find(FaceID) == ProjectInfo[RightImageID].end())
				{
					continue;
				}
				cv::RNG rng(cvGetTickCount());
				float Ran = rand() / double(RAND_MAX);//生成0~1的随机数
				cv::Scalar Color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

				vector<cv::Point> pts;
				pts.push_back(pair.second.P1);
				pts.push_back(pair.second.P2);
				pts.push_back(pair.second.P3);
				polylines(Photo1, pts, true, Color, 1, 8, 0);
				if (Ran <= FillProbability)
				{
					fillConvexPoly(Photo1, pts, Color, 8, 0);//填充该三角形
				}

				pts.clear();
				pts.push_back(ProjectInfo[RightImageID][FaceID].P1);
				pts.push_back(ProjectInfo[RightImageID][FaceID].P2);
				pts.push_back(ProjectInfo[RightImageID][FaceID].P3);
				polylines(Photo2, pts, true, Color, 1, 8, 0);
				if (Ran <= FillProbability)
				{
					fillConvexPoly(Photo2, pts, Color, 8, 0);//填充该三角形
				}
			}
			string OutputImageName = "ImagePair_" + to_string(CurrentImagePair + 1);
			cv::Mat TwoView;
			if (Config.IsDownSampling)
			{
				Photo1 = DownSampling(Photo1);
				Photo2 = DownSampling(Photo2);
				TwoView = MergeTwoView(Photo1, Photo2, 5);
				OutputImageName += "_DownSampling";
			}
			else
			{
				TwoView = MergeTwoView(Photo1, Photo2, 10);
			}
			OutputImageName += ".bmp";
			cv::imwrite(OutputImageName, TwoView);
			cout << "[" << CurrentImagePair + 1 << "/" << Config.TargetMatch.size() << "] Output match image: " << OutputImageName << endl;
		}
		CurrentImagePair++;
	}
	cout << "Complete!" << endl;
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
void LoadGenerateMatchImageConfig(string ConfigPath, CGenerateMatchImageConfig& GenerateMatchImageConfig)
{
	ifstream file(ConfigPath);
	file.imbue(locale(""));
	if (!file)
	{
		cout << "Error! " << ConfigPath << " does not exist!" << endl;
		exit(0);
	}
	string line, config, value;
	getline(file, line); //[GenerateMatchConfig]

	getline(file, line); //ObjDir=...
	Parse(line, config, value);
	value = regex_replace(value, regex("\\\\"), "/");
	replace(value.begin(), value.end(), '\\', '/');
	GenerateMatchImageConfig.ObjDir = value;

	getline(file, line); //CentralX=0
	Parse(line, config, value);
	GenerateMatchImageConfig.CentralX = stod(value);

	getline(file, line); //CentralY=0
	Parse(line, config, value);
	GenerateMatchImageConfig.CentralY = stod(value);

	getline(file, line); //CentralZ=0
	Parse(line, config, value);
	GenerateMatchImageConfig.CentralZ = stod(value);


	getline(file, line); //SvpPath=...
	Parse(line, config, value);
	value = regex_replace(value, regex("\\\\"), "/");
	replace(value.begin(), value.end(), '\\', '/');
	GenerateMatchImageConfig.SvpPath = value;

	getline(file, line); //IsDownSampling=...
	Parse(line, config, value);
	GenerateMatchImageConfig.IsDownSampling = (value == "1");

	getline(file, line); //IsMask=...
	Parse(line, config, value);
	GenerateMatchImageConfig.IsMask = (value == "1");

	getline(file, line); //TargetMatch=...\t...
	Parse(line, config, value);
	if (value.size() <= 3)return;

	istringstream stream(value);
	string LeftImageName, RightImageName;
	stream >> LeftImageName >> RightImageName;
	if (LeftImageName.size() <= 3 || RightImageName.size() <= 3)return;
	GenerateMatchImageConfig.TargetMatch.emplace_back(LeftImageName, RightImageName);

	while (getline(file, line))
	{
		if (value.size() <= 3)return;
		istringstream stream(line);
		string LeftImageName, RightImageName;
		stream >> LeftImageName >> RightImageName;
		if (LeftImageName.size() <= 3 || RightImageName.size() <= 3)return;
		GenerateMatchImageConfig.TargetMatch.emplace_back(LeftImageName, RightImageName);
	}
}
string GetFileName(string FilePath)
{
	FilePath = regex_replace(FilePath, regex("\\\\"), "/");
	std::replace(FilePath.begin(), FilePath.end(), '\\', '/');
	size_t pos = FilePath.find_last_of('/');
	string ImageName = (pos == string::npos ? FilePath : FilePath.substr(pos + 1));
	return ImageName;
}
bool DoBackProject(CAerialBlock& block, CModel& Model, size_t ImageID, size_t FaceID, CFace2D& Face2D)
{
	CAerialPhoto* Image = block.GetPhoto(ImageID);
	const CCamera* ImageCamera = Image->GetCamera();
	int Width, Height;
	CCCDCamera* CCDCamera = (CCCDCamera*)ImageCamera;
	CCDCamera->GetSensorSizeInPixel(Width, Height);
	Image->SetDimension(Width, Height);

	if (IsBlocked(Model, Image, FaceID))
	{
		Face2D = CFace2D();
		return false;
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
		Face2D = CFace2D(BackProjectP1, BackProjectP2, BackProjectP3);
		return true;
	}

	Face2D = CFace2D();
	return false;
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
cv::Mat DownSampling(cv::Mat& input)
{
	cv::Mat re = input.clone();
	while (true)
	{
		cv::pyrDown(re, re);
		if (re.rows <= 850 && re.cols <= 850)
			return re;
	}
}
cv::Mat MergeTwoView(cv::Mat& Photo1, cv::Mat& Photo2, int GapWidth)
{
	cv::Mat TwoView = cv::Mat::zeros(cv::Size(Photo1.cols + Photo2.cols + GapWidth, max(Photo1.rows, Photo2.rows)), Photo1.type());
	cv::Rect Roi(0, 0, Photo1.cols, Photo1.rows);
	Photo1.copyTo(TwoView(Roi));
	Roi = cv::Rect(Photo1.cols + GapWidth, 0, Photo2.cols, Photo2.rows);
	Photo2.copyTo(TwoView(Roi));
	return TwoView;
}



