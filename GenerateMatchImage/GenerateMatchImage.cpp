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

#include "../3rdParty/AerialBase/include/AerialBase.h"
using namespace std;

struct CGenerateMatchImageConfig
{
	string SvpPath;
	bool IsDownSampling;
	bool IsMask;
	vector<pair<string, string>> TargetMatch;
};


int main(int argc, char* argv[])
{
	cout << "Reading config file and .svp file..." << endl;




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






}









