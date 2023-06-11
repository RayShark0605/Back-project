#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <regex>

#include "../3rdParty/AerialBase/include/AerialBase.h"

using namespace std;

void LoadMergeBinConfig(string ConfigPath, string& SvpPath, vector<string>& BinPaths);
void LoadBinFile(string BinFilePath, vector<unordered_set<size_t>>& VisibleFaceIDs);
size_t GetMatchedNum(unordered_set<size_t>& Set1, unordered_set<size_t>& Set2);
int main(int argc, char* argv[])
{
	cout << "Reading config file and .svp file..." << endl;
	vector<string> BinPaths;
	string SvpPath;
	LoadMergeBinConfig("MergeBinConfig.ini", SvpPath, BinPaths);
	cout << "Detect " << BinPaths.size() << " .bin files!" << endl;
	CAerialBlock block;
	block.Reset();
	if (!block.LoadFromFile(SvpPath.c_str()))
	{
		cout << "Unable to open .svp file!" << endl;
		exit(0);
	}
	size_t ImagesNum = block.GetNumPhoto();
	cout << "ImagesNum: " << ImagesNum << endl;

	vector<unordered_set<size_t>> VisibleFaceIDs;
	for (size_t i = 0; i < BinPaths.size(); i++)
	{
		cout << "[" << i + 1 << "/" << BinPaths.size() << "] Loading " << BinPaths[i] << " ..." << endl;
		LoadBinFile(BinPaths[i], VisibleFaceIDs);
		cout << "[" << i + 1 << "/" << BinPaths.size() << "] Loading" << BinPaths[i] << " completed!" << endl;
	}

	cout << "Exporting merged information to .bin file..." << endl;
	ofstream ofs("Merged.bin", ios::binary);
	bool IsPartedBin = false;
	size_t MinImageID = 0, MaxImageID = ImagesNum - 1;
	ofs.write((char*)&IsPartedBin, sizeof(IsPartedBin));
	ofs.write((char*)&MinImageID, sizeof(MinImageID));
	ofs.write((char*)&MaxImageID, sizeof(MaxImageID));
	for (size_t i = 0; i < ImagesNum; i++)
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
				cout << "\r" << i + 1 << "/" << ImagesNum << " : " << Count << "/" << VisibleFaceIDs[i].size();
			}
		}
	}
	ofs.close();
	cout << endl << "Exporting completed!" << endl;

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

void LoadMergeBinConfig(string ConfigPath, string& SvpPath, vector<string>& BinPaths)
{
	ifstream file(ConfigPath);
	file.imbue(locale(""));
	if (!file)
	{
		cout << "Error! " << ConfigPath << " does not exist!" << endl;
		exit(0);
	}
	string line, config, value;
	getline(file, line); //[MergeBinConfig]

	getline(file, line); //SvpPath=...
	line = regex_replace(line, regex("\\\\"), "/");
	replace(line.begin(), line.end(), '\\', '/');
	size_t pos = line.find('=');
	if (pos == string::npos)
	{
		cout << "Error! " << line << " does not have \"=\"" << endl;
		exit(0);
	}
	SvpPath = line.substr(pos + 1);

	getline(file, line); //PartedBinPath=...
	line = regex_replace(line, regex("\\\\"), "/");
	replace(line.begin(), line.end(), '\\', '/');
	pos = line.find('=');
	if (pos == string::npos)
	{
		cout << "Error! " << line << " does not have \"=\"" << endl;
		exit(0);
	}
	BinPaths.push_back(line.substr(pos + 1));
	
	while (getline(file, line))
	{
		line = regex_replace(line, regex("\\\\"), "/");
		replace(line.begin(), line.end(), '\\', '/');
		BinPaths.push_back(line);
	}
}
void LoadBinFile(string BinFilePath, vector<unordered_set<size_t>>& VisibleFaceIDs)
{
	ifstream ifs(BinFilePath, ios::binary);

	bool IsPartedBin;
	ifs.read((char*)&IsPartedBin, sizeof(IsPartedBin));
	if (!IsPartedBin)
	{
		cout << BinFilePath << " is not a parted bin file!" << endl;
		exit(0);
	}

	size_t MinImageID, MaxImageID;
	ifs.read((char*)&MinImageID, sizeof(MinImageID));
	ifs.read((char*)&MaxImageID, sizeof(MaxImageID));
	cout << "[" << BinFilePath << "] Range of image IDs: " << MinImageID << "-" << MaxImageID << endl;
	size_t CurImagesNum = MaxImageID - MinImageID + 1;
	
	for (size_t CurOffset = 0; CurOffset < CurImagesNum; CurOffset++)
	{
		VisibleFaceIDs.push_back(unordered_set<size_t>());
		size_t ThisImageVisibleFaceNum;
		ifs.read((char*)&ThisImageVisibleFaceNum, sizeof(ThisImageVisibleFaceNum));
		for (size_t i = 0; i < ThisImageVisibleFaceNum; i++)
		{
			size_t FaceID;
			ifs.read((char*)&FaceID, sizeof(FaceID));
			VisibleFaceIDs[MinImageID + CurOffset].insert(FaceID);
			if (i % 1000 == 0)
			{
				cout << "\r" << CurOffset + 1 << "/" << CurImagesNum << " : " << i + 1 << "/" << ThisImageVisibleFaceNum;
			}
		}
	}
	ifs.close();
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