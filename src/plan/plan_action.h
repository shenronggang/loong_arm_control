/*
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================*/
#pragma once
#include"plan.h"
#include<array>
#include<map>

namespace Plan{

class actPlanClass final:public basePlanClass{
public:
	actPlanClass();
	void init(float dt)override final;
	void hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	void bye(const Nabo::inputStruct &inRef)override final;
	void log() override final;
private:
	void update();
	void plan();
	void dwdate(Nabo::outputStruct &outRef);
	void load(const string& file, vector<array<float,NMotMain>> &armData, vector<array<float,NMotFingerBoth>> &fingerData);
	void extractNameB4Dot(vector<string> recordTrajTxtName);
	int cntRcd;
	float *cmdJ,*cmdFinger;
	vecNf(NMotMain) jGreet,jRaiseL,jRaiseR,jWaveL,jWaveR;
	vecNf(NMotFingerBoth) fingerGreet,fingerRaiseL,fingerRaiseR,fingerWaveL,fingerWaveR,fingerFist;
	enum{
		No,Left,Right
	}wave;
	bool fistFlag;

	/*actions of loading data*/
	enum{
		Stop,Start
	}actn;

	// enum{
	// 	Swing, OneArmWave, DoubleArmWave
	// }actnChce;

	static const int NMotUpper=31;
	int actnChce;
	int armSwingSize, armSwingTime;
	vector<vector<array<float,NMotUpper>>> upperActnArr;
	vector<array<float,NMotUpper>> innerUpperArmActn;
	
	vector<vector<array<float,NMotMain>>> armActnArr;
	vector<vector<array<float,NMotFingerBoth>>> fingerActnArr;

	vector<array<float,NMotMain>> innerArmActn;
	vector<array<float,NMotFingerBoth>> innerFingerActn;

	string recordWholeTrajName;
	string recordWholeTrajKey;
	vector<string> recordTrajName;
	vector<string> recordTrajKey;
	vector<string> trajName;
	int recordTrajNum;
	vector<string> rltvTrajName;
	map<int,int> actKeyIds;

	/*actions of loading data*/
};
}//namespace
