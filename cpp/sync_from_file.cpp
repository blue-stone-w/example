/*
按行读取文件内容，并按行写入文件
*/

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std;

struct Pose 
{
	double time, timeDelay;
	double x, y, head;
};

void extractFile(string oriutmpath, vector<Pose>& rtksOut) 
{
	ifstream oriutmf;
	oriutmf.open(oriutmpath, ios::in);

	if (oriutmf) 
	{
		string line;
		int lineInd = 0;
		Pose poseLine;
		while (getline(oriutmf, line)) 
		{
			stringstream ss;
			ss << line;
			ss >> poseLine.time >> poseLine.x >> poseLine.y >> poseLine.head;
			rtksOut.push_back(poseLine);
			lineInd++;
		}
		oriutmf.close();
	}
	else 
	{
		cout << "error: " << oriutmpath  << endl;
	}
	return;
}

void saveFile(vector<Pose> rtksIn, string savePath) 
{
	ofstream oriutmf;
	oriutmf.open(savePath, ios::out);
	if (oriutmf) {
		for (unsigned lineInd = 0; lineInd < rtksIn.size(); lineInd++) 
		{
			oriutmf << setprecision(5) << rtksIn[lineInd].time << " " ;
			oriutmf << setprecision(3) << rtksIn[lineInd].timeDelay * 1000 << " " ;
			oriutmf << setprecision(2) << rtksIn[lineInd].x << " " << rtksIn[lineInd].y << " " << rtksIn[lineInd].head << " ";
			oriutmf << endl;
		}
		oriutmf.close();
	}
	else 
	{
		cout << "error: " << savePath << endl;
	}
}

Pose getDiff(Pose rtk1, Pose rtk2) 
{
	Pose temprtk;
	temprtk.timeDelay = rtk1.time - rtk2.time;
	temprtk.x = int((rtk1.x - rtk2.x)*100);
	temprtk.y = int((rtk1.y - rtk2.y)*100);
	temprtk.head = rtk1.head - rtk2.head;
	
	return temprtk;
}

void findSync(vector<Pose>& posetoSync, vector<Pose>& rtkFixed, vector<Pose>& rtkSynced) 
{
	// i: index of original pose; j: index of vecotr, from which find corresponding pose 
	for (int i = 0, j = 1; i < posetoSync.size(); i++) 
	{
		Pose temprtk;
		while (j < rtkFixed.size()) {
			if (posetoSync[i].time > rtkFixed[j].time) 
			{
				j++;
			}
			else 
			{
				break;
			}
		}
		if (2 * posetoSync[i].time > rtkFixed[j - 1].time + rtkFixed[j].time) // close to later one
		{ 
			temprtk = getDiff(posetoSync[i], rtkFixed[j]);
		}
		else // close to earlier one
		{
			temprtk = getDiff(posetoSync[i], rtkFixed[j - 1]);
		}
		temprtk.time = posetoSync[i].time;
		if (16 < temprtk.time && temprtk.time < 46)
		{
			rtkSynced.push_back(temprtk);
		}
	}
}
int main(int argc, char** argv) {
	// extract rtks from file to vector
	vector<Pose> orirtk, oriPose;

	extractFile("./orirtk.txt", orirtk);
	extractFile("./oriPose.txt", oriPose);

	cout <<"orirtk: "<< orirtk.size() << endl;
	cout << "oriPose: " << oriPose.size() << endl;

	// find nearest point from Pose for every point from lidar
	vector<Pose> poseDiff;
	findSync(oriPose, orirtk, poseDiff);
	cout << "poseDiff: " << poseDiff.size() << endl;
	saveFile(poseDiff, "./oridiff.txt");
	return 0;
}



